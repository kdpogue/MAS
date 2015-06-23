#include "includes.h"

/******************************************************************************
 * Private Data Type
 *****************************************************************************/
typedef struct {
	INT8U Data[255];
	INT8U DataSize;
}  PN532_PKT;

/******************************************************************************
 * MQX Resources 
 ******************************************************************************/
LWSEM_STRUCT RFIDSem;
void PN532IRQ_ISR(void * Input);
/******************************************************************************
 * MCU Specific Definitions
 *****************************************************************************/
#define TCF  0x80000000
#define IRQ 0x0008000
#define TFFF 0x02000000
#define EOQ 0x10000000
#define RFDF 0x00020000
#define RX_FIFO_COUNTER ((SPI1_SR & 0x000000F0) >> 4)
#define SS_ENABLE() GPIOE_PCOR = 0x10
#define SS_DISABLE() GPIOE_PSOR = 0x10
#define WAIT_FOR_IRQ() 	while(GPIOA_PDIR & IRQ) {}
#define WAIT_FOR_SPI_TX_RDY() while(!(TFFF & SPI1_SR)) {}
#define SEND_DATA(DATA) SPI1_PUSHR = ((INT32U) DATA) | 0x0001000
#define SEND_DATA_EOQ(DATA) SPI1_PUSHR = ((INT32U) DATA)| 0x08010000
#define WAIT_FOR_SPI_TX_DONE() 	while (!(SPI1_SR & EOQ)) {} SPI1_SR |= EOQ
#define READ_DATA(DATA)	DATA = SPI1_POPR
#define WAIT_FOR_SPI_RX_DONE() while(RX_FIFO_COUNTER != 1){}
#define DRAIN_SPI_RX_BUFFER(DATA) while(RX_FIFO_COUNTER) DATA = SPI1_POPR

/******************************************************************************
 * PN532 Specific Definitions
 *****************************************************************************/
#define RDY 0x01
#define CMD_START_LEN 6

/******************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

void SendCmd(PN532_PKT *Command);
INT8U ConfirmAck(void);
INT8U CalcDataCheckSum(PN532_PKT *Command);
INT8U GetResponse(PN532_PKT *result);
void ConfirmRDY(void);
void PN532Cmd(PN532_PKT *input, PN532_PKT *output);
INT32U GetNFCID(INT8U *data_in);
/******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/
INT32U ScanForCard(void);
void RFIDInit(void);

/******************************************************************************
 * Written By: Kevin Pogue
 * RFIDInit()
 * Initializes the SPI to transmit and receive from the PN532 at 3 MHz (must be
 * less than 5 MHz), CPOL = 0, CPHA = 0, master mode, no continuous clock,
 * slave select active low for entire transmission frame.
 * See NXP Application Note PN532 C106 for SPI waveforms required by PN532
 * Parameters
 * 	None  
 *****************************************************************************/

void RFIDInit(void) {
	SIM_SCGC5 |= 0x2200; 		/* Turn PORTA and PORTE clock on */
	SIM_SCGC6 |= 0x2000;		/* Turn SPI module clock on */
	PORTA_PCR15 = 0x000A0100;	/* Set IRQ pin to GPIO input w/ interrupt on 
								falling edge */
	NVICICPR2 |= 1 << (87 % 32);
	NVICISER2 |= 1 << (87 % 32);
	PORTE_PCR4 = 0x00000103;	/* Set SPI1_PCS 0 to GPIO mode */
	GPIOE_PDDR |= 0x00000010;	/* Set PCS0 to output */
	SS_DISABLE();				/* Set PCS0 high */
	PORTE_PCR3 |= 0x00000200;	/* Set PORTE SPI1 bits to SPI mode */
	PORTE_PCR2 |= 0x00000200;	/* Set PORTE SPI1 bits to SPI mode */
	PORTE_PCR1 |= 0x00000200;	/* Set PORTE SPI1 bits to SPI mode */
	SPI1_MCR = 0x800F0C00;		/* Master mode, CS_L, use buffers, clear 
	 	 	 	 	 	 	 	 	buffers no edges between samples */
	SPI1_TCR = 0x0;				/* Reset transfer count for debugging */
	SPI1_CTAR0 = 0x39010000;	/* 8 bit transfer, CPOL, CPHA = 0, no delays */
	_lwsem_create(&RFIDSem, 0);
	_int_install_isr(0x67, (INT_ISR_FPTR) PN532IRQ_ISR, (void *) 0);
}

/******************************************************************************
 * Written By: Kevin Pogue
 * ScanForCard()
 * Runs InListPassiveTarget PN532 command and returns the ID of the card found 
 * Parameters
 * 	None
 * Return
 * 	Returns the NFCID of the ID card found and 0 if no card is found.  
 *****************************************************************************/
INT32U ScanForCard(void){
	PN532_PKT InListPassiveTarget;
	PN532_PKT SAMConfiguration;
	PN532_PKT result;
	
	InListPassiveTarget.Data[0] = 0xD4;
	InListPassiveTarget.Data[1] = 0x4A;
	InListPassiveTarget.Data[2] = 0x01;
	InListPassiveTarget.Data[3] = 0x00;
	InListPassiveTarget.DataSize = 4;
	
	SAMConfiguration.Data[0] = 0xD4;
	SAMConfiguration.Data[1] = 0x14;
	SAMConfiguration.Data[2] = 0x01;
	SAMConfiguration.DataSize = 3;
	
	PN532Cmd(&SAMConfiguration, &result);
	PN532Cmd(&InListPassiveTarget, &result);
	return (GetNFCID(result.Data));
}

/******************************************************************************
 * Written By: Kevin Pogue
 * GetNFCID()
 * Extracts NFCID from InListPassiveTarget result command data array.
 * Parameters
 * 	data_in: Pointer to data packet received from InListPassiveTarget command,
 * 			 behavior for other INT8U * is undefined
 * Return
 * 	Returns the NFCID  as a 32 bit unsigned integer	 
 *****************************************************************************/
INT32U GetNFCID(INT8U *data_in){
	INT32U id = 0;
	if (data_in[2] == 1){ /* Number of targets found == 1, there is a card */
		id = ((INT32U) data_in[8] << 24) | ((INT32U) data_in[9] << 16) |
			 ((INT32U) data_in[10] << 8)| ((INT32U) data_in[11]);
	} else {}
	return id;
}

/******************************************************************************
 * Written By: Kevin Pogue
 * PN532Cmd()
 * Sends a PN532 command to the PN532, receives and checks the acknowledgement 
 * and places the PN532 output data in a PN532_PKT data type
 * Parameters
 * 	input: Contains a pointer to the command to be sent
 * 	output: A pointer to the response sent from the PN532, any values in this 
 * 			parameter will be overwritten
 *****************************************************************************/
void PN532Cmd(PN532_PKT *input, PN532_PKT *output) {
	SendCmd(input);
	WAIT_FOR_IRQ();
	ConfirmAck();
	WAIT_FOR_IRQ();
	GetResponse(output);
}

/******************************************************************************
 * Written By: Kevin Pogue
 * SendCmd()
 * Sends a PN532 command to the PN532 by SPI
 * Parameters
 * 	Command: A pointer to the data to be sent to the PN532.  
 *****************************************************************************/
void SendCmd(PN532_PKT *Command){
	INT8U start_frame[CMD_START_LEN]= 
		{ 	0x01,						/* Data write */  
			0x00, 						/* Preamble */
			0x00, 						/* Start of Packet 1*/
			0xff, 						/* Start of Packet 2*/
			Command -> DataSize,  		/* Length */
			(0 - Command -> DataSize)  	/* Length Checksum */
		};
	INT8U dcs =  CalcDataCheckSum(Command);
	INT32U data_out;
	INT8U i;
	SS_ENABLE();
	/* Send start_frame */
	for (i = 0; i < CMD_START_LEN; i++) {
		WAIT_FOR_SPI_TX_RDY()
		SEND_DATA(start_frame[i]);
	}
	/* Send command */
	for (i = 0; i < Command -> DataSize; i++) {
		WAIT_FOR_SPI_TX_RDY()
		SEND_DATA(Command -> Data[i]);
	}
	WAIT_FOR_SPI_TX_RDY()
	/* Send data checksum */
	SEND_DATA(dcs); 
	WAIT_FOR_SPI_TX_RDY()
	/* Send postamble */
	SEND_DATA_EOQ(0x00);
	WAIT_FOR_SPI_TX_DONE();
	SS_DISABLE();
	DRAIN_SPI_RX_BUFFER(data_out);
}


/******************************************************************************
 * Written By: Kevin Pogue
 * CalcDataCheckSum()
 * Calculates the value of the data checksum used in PN_532 transmissions
 * Parameters
 * 	Command: A pointer to the PN532_PKT whose checksum is to be calculated
 * Return:
 * 	The value of the checksum calculation  
 *****************************************************************************/
INT8U CalcDataCheckSum(PN532_PKT *Command) {
	INT8U cs = 0; 	/* Checksum result*/
	INT8U i;		/* Counter */
	for (i = 0; i < (Command -> DataSize); i++) {
		cs += Command -> Data[i]; 
	}
	return (0 -cs);
}

/******************************************************************************
 * Written By: Kevin Pogue
 * ConfirmAck()
 * Receives and checks the acknowledge frame sent from the PN532 after a packet
 * has been received.
 * Parameters
 * 	None
 * Return
 * 	Returns TRUE if acknowledge received, false if NACK or other data received 
 *****************************************************************************/
INT8U ConfirmAck(void) {
	INT8U i;
	INT32U ack_received[7]; 
	INT8U acknowledge_confirmed = TRUE;		
	const INT8U ack_good[7] = {0x01, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00};
	ConfirmRDY();
	/* Shift out 0's to receive data from PN532 */
	SS_ENABLE();
	WAIT_FOR_SPI_TX_RDY();
	SEND_DATA(0x03); /* Data Read */
	WAIT_FOR_SPI_RX_DONE();
	READ_DATA(ack_received[0]);
	for (i = 1; i < 7; i++){
		WAIT_FOR_SPI_TX_RDY();
		if (i != 6){
			SEND_DATA(0x0000);	/* Shift data in */
		} else {
			SEND_DATA_EOQ(0x0000); 	/* Shift data in w/ EOQ bit */
		}
		WAIT_FOR_SPI_RX_DONE();
		READ_DATA(ack_received[i]);
	}
	WAIT_FOR_SPI_TX_DONE();
	SS_DISABLE();
	for (i = 0; i < 7; i++){
		if (ack_received[i] != ack_good[i]) {
			acknowledge_confirmed = FALSE;
		} else {}
	}
	return acknowledge_confirmed;
}

/******************************************************************************
 * Written By: Kevin Pogue
 * GetResponse()
 * Gets Response frame from PN532 by SPI, verifies checksums and stores data in 
 * result parameter
 * Parameters
 * 	Result: A pointer to where the data sent from the PN532 is to be stored
 * Return
 * 	Returns TRUE if no error, false if there was an error in transmission  
 *****************************************************************************/
INT8U GetResponse(PN532_PKT *result){
	const INT8U start_frame[CMD_START_LEN - 2]= 
		{ 	0x01,						/* RDY */  
			0x00, 						/* Preamble */
			0x00, 						/* Start of Packet 1*/
			0xff, 						/* Start of Packet 2*/
		};
	INT32U rx_data = 0;
	INT8U i;
	INT8U rx_error = FALSE;
	ConfirmRDY();
	DRAIN_SPI_RX_BUFFER(rx_data);
	SS_ENABLE();
	for (i = 0; i < CMD_START_LEN; i++) {
		WAIT_FOR_SPI_TX_RDY();
		if (i == 0) {
			 SEND_DATA(0x0003); /* Data Read */
		} else {
			SEND_DATA(0x0000);	/* Shift data in */
		}
		WAIT_FOR_SPI_RX_DONE();
		READ_DATA(rx_data);	
		if (i < CMD_START_LEN - 2){
			if (rx_data != start_frame[i]) {
				rx_error = TRUE;
			} else {}
		} else if (i == CMD_START_LEN -2) {
			result -> DataSize = rx_data;
		} else if (i == CMD_START_LEN - 1) {
			if ((((INT8U) rx_data) + result->DataSize) != 0) {/* Length checksum error */
				rx_error = TRUE;
			}
		} else {
			rx_error = TRUE; /* Should never get here, something went wrong */
		}
	}
	for (i = 0; i < result->DataSize; i++) {
		WAIT_FOR_SPI_TX_RDY();
		SEND_DATA(0x0000); /* Shift data in */
		WAIT_FOR_SPI_RX_DONE();
		READ_DATA(result->Data[i]);
	}
	WAIT_FOR_SPI_TX_RDY();
	SEND_DATA(0x0000); /* Shift data in */
	WAIT_FOR_SPI_RX_DONE();
	READ_DATA(rx_data);
	if (rx_data != CalcDataCheckSum(result)) {
		rx_error = TRUE;
	} else {}
	WAIT_FOR_SPI_TX_RDY();
	SEND_DATA_EOQ(0x0000); /* Shift data in */
	WAIT_FOR_SPI_RX_DONE();
	READ_DATA(rx_data);
	if (rx_data != 0){
		rx_error = TRUE;
	} else {}
	WAIT_FOR_SPI_TX_DONE();
	SS_DISABLE();
	return (!rx_error); /* Return TRUE if no error, FALSE if error */
}

/******************************************************************************
 * Written By: Kevin Pogue
 * ConfirmRDY()
 * Sends Status Read (0x02) and checks for RDY response (0x01) from PN532,
 * loops until RDY received
 * Parameters
 * 	None
 * 	Must only be used when PN532 has received a command and will be ready to 
 * 	transmit, or an infinite loop in this function is very likely
 *****************************************************************************/
void ConfirmRDY(void){
	INT32U rx_data = 0;
	while (rx_data != RDY) {
		SS_ENABLE();
		WAIT_FOR_SPI_TX_RDY();
		SEND_DATA(0x0002); /* Status Reading */
		while (RX_FIFO_COUNTER != 0) { /* Drain Rx FIFO */
			READ_DATA(rx_data);
		}
		WAIT_FOR_SPI_TX_RDY();
		SEND_DATA_EOQ(0x0000);	/* Look for RDY bit, EOQ */
		READ_DATA(rx_data);
		WAIT_FOR_SPI_TX_DONE();
		SS_DISABLE();
	}
	DRAIN_SPI_RX_BUFFER(rx_data);
}

void PN532IRQ_ISR(void * Input){
	_lwsem_post(&RFIDSem);
}
