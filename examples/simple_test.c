/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "ethercat.h"
#include "osal.h"
#include "oshw.h"
#include "nuc980.h"
/******************************************************************************
* hex dump 
*/
#define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
static void hex_dump(const rt_uint8_t *ptr, rt_size_t buflen)
{
    unsigned char *buf = (unsigned char *)ptr;
    int i, j;

    RT_ASSERT(ptr != RT_NULL);

    for (i = 0; i < buflen; i += 16)
    {
        rt_kprintf("%08X: ", i);

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%02X ", buf[i + j]);
            else
                rt_kprintf("   ");
        rt_kprintf(" ");

        for (j = 0; j < 16; j++)
            if (i + j < buflen)
                rt_kprintf("%c", __is_print(buf[i + j]) ? buf[i + j] : '.');
        rt_kprintf("\n");
    }
}


static unsigned char IOmap[4096];
typedef struct  __attribute__((__packed__))
{
	unsigned char  mode_byte;
	unsigned short control_word;
	long           dest_pos;
	unsigned short error_word;
	unsigned short status_word;
	long  cur_pos;
}SERVO_DATA_T;

typedef struct
{
	SERVO_DATA_T servo_data[3];
}SERVOS_T;

static SERVOS_T *servos = (SERVOS_T *)IOmap;

static void view_slave_data()
{
	hex_dump(IOmap,32); 
}

static void echo_time()
{
	struct timeval tp;
	osal_gettimeofday(&tp, 0);
	printf("****cur time = %ud,%03ld,%03ld(us)\n", tp.tv_sec,tp.tv_usec/1000,tp.tv_usec%1000);
}

#define EC_TIMEOUTMON 500
static int expectedWKC;
static volatile int wkc;


static int safe_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, int size, void *data)
{
	int wkc, cnt=0;
	do{
		wkc = ec_SDOwrite(Slave, Index, SubIndex, FALSE, size, data, EC_TIMEOUTRXM);
		cnt++;
	}while(wkc<=0 && cnt<10);
	return wkc;
}
static int safe_SDCwrite_b(uint16 Slave, uint16 Index, uint8 SubIndex, uint8 b)
{
	return safe_SDOwrite(Slave, Index, SubIndex, 1, &b);
}
static int safe_SDCwrite_w(uint16 Slave, uint16 Index, uint8 SubIndex, uint16 w)
{
	return safe_SDOwrite(Slave, Index, SubIndex, 2, &w);
}
static int safe_SDCwrite_dw(uint16 Slave, uint16 Index, uint8 SubIndex, uint32 dw)
{
	return safe_SDOwrite(Slave, Index, SubIndex, 4, &dw);
}

static int safe_SDOread(uint16 Slave, uint16 Index, uint8 SubIndex, int size, void *data)
{
	int wkc, cnt=0;
	do{
		wkc = ec_SDOread(Slave, Index, SubIndex, FALSE, &size, data, EC_TIMEOUTRXM);
	}while(wkc<=0 && cnt<10);
	return wkc;
}
static int safe_SDOread_b(uint16 Slave, uint16 Index, uint8 SubIndex, uint8 b)
{
	return safe_SDOread(Slave, Index, SubIndex, 1, &b);
}
static int safe_SDOread_w(uint16 Slave, uint16 Index, uint8 SubIndex, uint16 w)
{
	return safe_SDOread(Slave, Index, SubIndex, 2, &w);
}
static int safe_SDOread_dw(uint16 Slave, uint16 Index, uint8 SubIndex, uint32 dw)
{
	return safe_SDOread(Slave, Index, SubIndex, 4, &dw);
}


static void viewSDO(uint16_t slave, uint16_t index, uint16_t subindex, int bytes)
{
	uint32_t dw = 0;
	int wkc;
	safe_SDOread(slave, index, subindex, bytes, &dw);
	printf("SDO read=%s, SDO[0x%04x.%02x] = 0x%08x\n", wkc?"success":"fail",index, subindex, dw);
}

static void process_data_config()
{
	uint8_t     ind;
	
	for(int slave = 1; slave <= *ecx_context.slavecount; slave++)
	{
		//rpdo------------
		//1c12.0
		safe_SDCwrite_b(slave, 0x1c12, 0, 0);
		safe_SDCwrite_w(slave, 0x1c12, 1, 0x1600);
		
		//1600
		ind = 0;
		safe_SDCwrite_b(slave, 0x1600, 0, 0);
		safe_SDCwrite_dw(slave, 0x1600, ++ind, htoel(0x60600008));//6060h(????)
		safe_SDCwrite_dw(slave, 0x1600, ++ind, htoel(0x60400010));//6040h(???)
		safe_SDCwrite_dw(slave, 0x1600, ++ind, htoel(0x607a0020));//607Ah(????)
		safe_SDCwrite_b(slave, 0x1600, 0, ind);

		//1c12.0
		safe_SDCwrite_b(slave, 0x1c12, 0, 1);
		
		//tpdo-------------
		//1c13.0
		safe_SDCwrite_b(slave, 0x1c13, 0x00, 0);
		safe_SDCwrite_w(slave, 0x1c13, 0x01, 0x1a00);
		
		//1a00
		ind = 0;
		safe_SDCwrite_b(slave, 0x1a00, 0, 0);
		safe_SDCwrite_dw(slave, 0x1a00, ++ind, htoel(0x603F0010));//603Fh(???)
		safe_SDCwrite_dw(slave, 0x1a00, ++ind, htoel(0x60410010));//6041h(???)
		safe_SDCwrite_dw(slave, 0x1a00, ++ind, htoel(0x60640020));//6064h(????)
		safe_SDCwrite_b(slave, 0x1a00, 0, ind);

		//1c13.0
		safe_SDCwrite_b(slave, 0x1c13, 0, 1);
		
		safe_SDCwrite_b(slave, 0x6060, 0, 1);
		//viewSDO(slave, 0x6060, 0, 1);
	}
}


static void servo_switch_op()
{
	int sta;
	for(int slave = 1; slave <= *ecx_context.slavecount; slave++)
	{
		int idx = slave - 1;
		sta = servos->servo_data[idx].status_word & 0x3ff;
		//printf("cia402 servo status = %04x\n", sta);
		//clear fault
		if(servos->servo_data[idx].status_word & 0x8){
			servos->servo_data[idx].control_word = 0x80;
			continue;
		}

		//swtich servo stattus, ref cia402 
		switch(sta)
		{
			case 0x250:
			case 0x270:
				servos->servo_data[idx].control_word = 0x6;
				break;
			case 0x231:
				servos->servo_data[idx].control_word = 0x7;
				break;
			case 0x233:
				servos->servo_data[idx].control_word = 0xf;
				break;
			default:
				//servos->servo_data[idx].control_word = 0x6;
				break;
		}
		//printf("cia402 servo control = %04x\n", servos->servo_data[idx].control_word);
	}
	
}
static void servo_switch_idle()
{
	for(int slave = 1; slave <= *ecx_context.slavecount; slave++)
	{
		servos->servo_data[slave-1].control_word = 0x0;
	}
}

static void is620n_test(char *ifname)
{
	osal_timer_init();

	//we do etherCAT status switch manually
	ecx_context.manualstatechange = 1;
	
	printf("========================\n");
	printf("sv660 config\n");
	echo_time();
	
	if (ec_init(ifname))
	{
		printf("ec_init on %s succeeded.\n",ifname);
				
		//request INIT state for all slaves 
		printf("\nRequest init state for all slaves\n");
		ec_slave[0].state = EC_STATE_INIT;
		ec_writestate(0);
		
		/* wait for all slaves to reach SAFE_OP state */
		ec_statecheck(0, EC_STATE_INIT,  EC_TIMEOUTSTATE * 3);
		if (ec_slave[0].state != EC_STATE_INIT ){
			printf("Not all slaves reached init state.\n");
			ec_readstate();
			for(int i = 1; i<=ec_slavecount ; i++){
				if(ec_slave[i].state != EC_STATE_INIT){
					printf("Slave %d State=0x%2x StatusCode=0x%04x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
				}
			}
		}
		echo_time();
		
		wkc = ec_config_init(0/*usetable*/);

		if (wkc > 0)
		{
			//DC config
			ec_configdc();
			ec_dcsync0(1, TRUE, 5000000, 50); // SYNC0 on slave 1
			while(EcatError) printf("%s", ec_elist2string());
			printf("%d slaves found and configured.\n",ec_slavecount);
		
			// request pre_op for slave 
			printf("\nRequest pre_op state for all slaves\n");
			ec_slave[0].state = EC_STATE_PRE_OP;
			ec_writestate(0);
			
			ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 3);
			if (ec_slave[0].state != EC_STATE_SAFE_OP ){
				printf("Not all slaves reached pre operational state.\n");
				ec_readstate();
			}
			
			//reset fault
			safe_SDCwrite_w(1,0x200d, 0x2, 1);	
			safe_SDCwrite_w(1,0x200d, 0x2, 0);	
			
		
			//now in pre_op
			process_data_config(); //config tpdo/rpdo
			
			//config fmmu
			ec_config_map(IOmap);
			
			
			// request safe_op for slave
			ec_slave[0].state = EC_STATE_SAFE_OP;
			ec_writestate(0);
			
			//safe-op
			expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
			printf("Calculated workcounter %d\n", expectedWKC);
			
			/* wait for all slaves to reach SAFE_OP state */
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 3);
			if (ec_slave[0].state != EC_STATE_SAFE_OP ){
				printf("Not all slaves reached safe operational state.\n");
				ec_readstate();
				for(int i = 1; i<=ec_slavecount ; i++){
					if(ec_slave[i].state != EC_STATE_SAFE_OP){
						printf("Slave %d State=0x%2x StatusCode=0x%04x : %s\n", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
					}
				}
			}else{
				//servo in csp mode
				for(int slave = 1; slave <= *ecx_context.slavecount; slave++)
					servos->servo_data[slave-1].mode_byte = 8; 
				
				//op status
				printf("Request operational state for all slaves\n");
				expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
				printf("Calculated workcounter %d\n", expectedWKC);
				// send one valid process data to make outputs in slaves happy
				ec_send_processdata();
				wkc = ec_receive_processdata(EC_TIMEOUTRET*3);
				//printf("--->workcounter %d\n", wkc);
				//view_slave_data();
				
				// request OP state for all slaves 
				ec_slave[0].state = EC_STATE_OPERATIONAL;
				ec_writestate(0);

				int chk = 200;
				// wait for all slaves to reach OP state 
				do
				{
					servo_switch_op();
					ec_send_processdata();
					ec_receive_processdata(EC_TIMEOUTRET);
					printf("--->workcounter %d\n", wkc);
					ec_statecheck(0, EC_STATE_OPERATIONAL, 2000);
				}
				while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

				//cyclic process data tx/rx
				if (ec_slave[0].state == EC_STATE_OPERATIONAL )
				{
					printf("<<<Operational>>> state reached for all slaves.\n");

					osal_timert t;
					osal_timer_start(&t, 1000);

					// cyclic loop
					for(int i = 1; i <= 10000; i++)
					{
						servo_switch_op();
						if(servos->servo_data[0].control_word==7){
							servos->servo_data[0].dest_pos = servos->servo_data[0].cur_pos;
						}
						if(servos->servo_data[0].control_word==0xf){
							servos->servo_data[0].dest_pos -= 3000;
						}
						
						//wait for timeout
						while(osal_timer_is_expired(&t)==FALSE);
						osal_timer_start(&t, 1000);

						ec_send_processdata();
						wkc = ec_receive_processdata(EC_TIMEOUTRET);

						if(wkc >= expectedWKC){
							//printf("~~~~WKC %d \n", wkc);
						}else if(wkc <=0 ){
							printf("tx/rx Error.\n");
							break;
						}
						//osal_usleep(1000);
					}
				}
				else
				{
					printf("Not all slaves reached operational state.\n");
					ec_readstate();
					for(int i = 1; i<=ec_slavecount ; i++)
					{
						if(ec_slave[i].state != EC_STATE_OPERATIONAL)
						{
							printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
						}
					}
				}
				
				//request INIT state for all slaves 
				printf("\nRequest init state for all slaves\n");
				ec_slave[0].state = EC_STATE_INIT;
				ec_writestate(0);
			}
		} else {
			printf("No slaves found!\n");
		}
		
		echo_time();
		printf("End soem, close socket\n");
		
		// stop SOEM, close socket 
		ec_close();		
	}else{
		printf("ec_init on %s failed.\n",ifname);
	}
	
	printf("========================\n");
	view_slave_data();
}


static void ec_simpletest(int argc, char **argv)
{
	if(argc<2){
		printf("usage : ectest e0\n");
		return;
	}
	is620n_test(argv[1]);
}

MSH_CMD_EXPORT(ec_simpletest, EtherCAT cia402 servo control sample);
