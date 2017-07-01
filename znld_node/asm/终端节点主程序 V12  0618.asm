;                 ����·���ն˽ڵ������� ��C8051F410��11.0592MHz    2017.05.10  LHB
             
						  ;Ӳ���汾 SMART-L-T01 V1.2B
             ;*************************************************
             ;���������ݣ�SPI������չоƬ��չ2�����ڣ�����A�õ��ܼ�⣬����B�������ݼ��ģ��
						 ;ͨ�����߽����ϱ����ݣ�����
             ;���2·PWM�źţ�������5V��Ƶ��500Hz
						 ;ģ�������PWM�˲�������1-10V��������Ƶ��
             ;���2·�̵���
						 ;I2C���߹� EEPROM������ID��
						 ;P1.7   AUX
						 ;30H��ʼΪ��������
						 ;4B��ʼΪ����֡
						;2016-6-14 ֡����Ϊ22�ֽ�
						 ;65H��ʼ6�ֽ�Ϊ����ID�ţ��������EEPROM��
						 ;���ӹ㲥���ݱ�־���㲥ʱ���ط�
							;2017-7-1 ���ӿ���ͨ��
             ;*************************************************
						

						AUX	 		BIT		P1.7       ;����ģ��״̬��⣬=1����
						RESIGN	EQU   22H.0      ;����֡�����=1���ֽ����ɴ���
						CRCER   EQU   22H.1      ;CRC������յĶԱȴ����ǣ�=1���д�Ҫ����Ӧ��
   					DATERR  EQU		22H.2      ;���ݽ���ʱ����=1�д�Ҫ����Ӧ��
						FNUM		EQU   7BH        ;֡��ˮ��
 						FHFIND	EQU		22H.5	
						BCSIGN	EQU		22H.6      ;�㲥��־��=1Ϊ�㲥����������

         $INCLUDE (C8051F410.inc)
            CSEG  AT 0
						SJMP  PROG
						ORG   0023H
						LJMP	URT0       ;�����ж����
            ORG   0030H
      PROG: MOV   PCA0MD, #00H      ;410��ʼ����disable watchdog timer
           ;���ⲿ����               �ڲ�����Ƶ�ʵ�ʱ��Ϊ24.5MHZ/128
            MOV   OSCXCN, #67H      ;�ⲿ������11.0592MHZ, �ڲ�����Ϊ24.5/128(MHZ)
						MOV   R7, #70           ;wait 1ms
            DJNZ  R7, $
    OSCwai: MOV   A, OSCXCN         ;��ѯ XTLVLD-->1
            JNB   ACC.7, OSCwai
            NOP					  
						NOP
						;I/O������
            MOV   P0MDIN, #0FCh     ;P0����ģ������
            MOV   P0MDOUT, #0FFH     ;P0.7 �������
            MOV   P1MDIN, #0FCh     ;P1.0,P1.1Ϊģ�����룬P1.2-P1.7Ϊ��������
            MOV   P1MDOUT, #0FCh     ;P1.2~P1.7 �������
   	        MOV   P2MDIN, #0FFh     ;P2 ��Ϊ��������
            MOV   P2MDOUT, #127     ;P2.0-P2.6 �������
   	        MOV   XBR0, #07h        ;UART,SPI,IIC,CEX0,CEX1�ӵ��˿�
            MOV   P1SKIP, #03h      ;P1.0-P1.1 ����
						MOV   P0SKIP,#03H       ;P0.0,P0.1  d/a����
            MOV   XBR1, #42h        ;���濪��ʹ�� 
            MOV   CLKSEL, #31H      ;ϵͳʱ�����ⲿʱ��11.0592MHZ��
						MOV		CLKMUL,#02H
						;MOV		OSCICN,#86H

            ;�ο���ѹ����
						MOV   REF0CN,#18H     ;vdd���ο������2.25v

           ;��ʱ���ʹ�������
            MOV   TCON,#10H
						MOV   TMOD, #22H        ;T0,T1 8λ�Զ���װ,T0����IIC��T1���ڴ��ڲ�����
						MOV   CKCON, #5         ;T0,T1��ϵͳʱ�ӣ����ڲ���������						
						MOV   TL0, #0A4H        ;IIC=40k
            MOV   TH0, #0A4H
						MOV	  SMB0CF,#80H;
            MOV   TH1, #70H         ;UART0������ 9600b/S,spi=6400
            MOV   TL1, #70H
						
						;SPI  ���ã�������չ��
						MOV   SPI0CFG,#40H;      4������ģʽ
    				MOV   SPI0CN,#05H;
    				MOV		SPI0CKR,#0B7H;      SCK=30K
						
						;DAC   ����룬д���ֽ�����,����0.5mA
						MOV   IDA0CN,#0F1H
						MOV   IDA1CN,#0F1H
						NOP
						MOV		IDA0L,#0
						MOV		IDA1L,#0
						MOV		IDA0H,#80H      ;�м�������
						MOV		IDA1H,#80H 
						NOP     ;

						;PWM ����
						MOV		PCA0CN,#40h       ;ʹ��T0�������ʱ�ӣ�40K��,����6.4ms
						ANL   PCA0MD,#0BFH
					;	MOV		PCA0MD,#04H
						MOV	  PCA0MD,#0       ;�ؿ��Ź���PCAʱ��=ϵͳʱ��/12 ��ģ�����ʱ��
    				MOV		PCA0CPM0,#42H  ;     8λPWM���
    				MOV		PCA0CPM1,#42H  ;
    				MOV		PCA0L,#80H
						MOV		PCA0H,#80H
						NOP
    				MOV		PCA0CPL0,#0FFH  ;  =FFFFH,���=0V  
    				MOV		PCA0CPH0,#0FFH  ;  PCA0��Сռ�������д���ֽ�����
					  MOV   PCA0CPL1,#0FFH  ;
    				MOV	  PCA0CPH1,#0FFH  ;
						NOP

						;CRC32��ʼ��
						MOV		CRC0CN,#1DH       ;CRC16,��ʼ=FF,���CRC0DAT�Ӹ��ֽڿ�ʼ��						 
					           
            LCALL WAT               ;
     BEGIN: MOV   SP, #0EFH         ;init stack pointer
            MOV	  R0, #255	        ;RAM���
	          CLR   A
     CLEAR: MOV   @R0, A
            DJNZ  R0, CLEAR
						NOP
						MOV		R0,#30H           ;���ջ�������ʼ��ַ
            SETB  TR0 
            SETB  TR1               ;UART0 ����
            MOV   SCON0, #10H       ;8λUART������ʹ��
						MOV   IE,#90H           ;���ڿ��ж�
						SETB	P2.0           ;�̵���0��
						SETB  P2.1           ;�̵���1��

 						; ������չоƬ��ʼ��
		;				MOV		SPI0DAT,#18H    ;��1�����ܣ�= 4800bps��8N1����2��������=9600��8N1
		;				MOV		SPI0DAT,#80H    ;��LCRд80H
		;				MOV		SPI0DAT,#00H
		;				MOV		SPI0DAT,#18H    ;��DLLд18H    4800bps
		;				MOV		SPI0DAT,#01H
		;				MOV		SPI0DAT,#00H    ;��DLHд0H
		;				MOV		SPI0DAT,#18H
		;				MOV		SPI0DAT,#0BBH   ;��LCRдBBH��8N1
		;				MOV		SPI0DAT,#18H
		;				MOV		SPI0DAT,#03H		;��LCRд03H
		;				MOV		SPI0DAT,#08H
		;				MOV		SPI0DAT,#01H    ;IERд01H���������ж�
		;				NOP
		;				NOP
K1:	;				MOV		SPI0DAT,#00H
		;				MOV		SPI0DAT,#0AAH
		;				NOP
		;				NOP
		;				NOP
		;				NOP
           ; SJMP  K1

						NOP
						CLR			RESIGN    ;22H.0        ;RESIGN   ֡���ձ��
						CLR			CRCER     ;22H.1          ;CRCER    CRC������
						CLR			DATERR    ;22H.2        ;DATERR   ���ݽ���������
						CLR			22H.5
						CLR			22H.6
						MOV			FNUM,#0
						CLR			BCSIGN
						SETB		EA
						NOP
						CLR			P2.0
						NOP	
						CLR			P2.1
						MOV			PCA0CPL0,#0H  ;  =00H,���=10V  
    				MOV			PCA0CPH0,#0H  ;  PCA0���ռ�������д���ֽ�����
					  MOV  	  PCA0CPL1,#0H  ;
    				MOV	 	  PCA0CPH1,#0H  ;
						NOP

          ;������  
MAIN: 			NOP
					  
					  NOP
						JNB			RESIGN,$
						;CLR			RESIGN
						NOP
					
						LCALL		CRCD
						MOV			R0,#30H
						MOV			CRC0CN,#1DH
					  JB			CRCER,MAIN
						NOP
						LCALL		DAPP
						MOV			R0,#30H									
				
						SETB		EA
            SJMP  	MAIN					
						

					

      ;���ڽ����жϴ���
URT0:       NOP
          	CLR     EA
						PUSH    ACC
			 			MOV     A, SBUF0
						CLR			RI0
						JB			FHFIND,UU3
						CJNE		A,#55H,UU1
						JNB			RI0,$
						MOV			A,SBUF0
						CLR			RI0
						CJNE		A,#55H,UU1
						SETB		FHFIND
						MOV			30H,#55H
						INC			R0
						MOV			31H,#55H
						MOV			R0,#32H
						;INC			R0
						SJMP		UU1
            
UU3:				MOV			@R0,A
						INC			R0
						CJNE		R0,#46H,UU1
						SETB		RESIGN             ;����22�ֽڣ�=1 ���жϲ��ټ�������
						MOV			R0,#30H
						CLR			EA
						SJMP		UU2
						NOP

UU1:				SETB		EA
UU2:				POP     ACC
						NOP
						RETI

   ;==================�ӳ�����===================
           ;---------------------------------------------
           
			  ;���� ���� CRC16
CRCD:				MOV		CRC0CN,#1DH
						MOV			R7,#14H          ;����ǰ20�ֽ�
						MOV			R0,#30H
UC4:				MOV			A,@R0
						MOV			CRC0IN,A
						INC			R0
						DJNZ		R7,UC4
						NOP	    
						MOV			R0,#44H
						CLR			CY
						MOV			A,CRC0DAT        ;CRC���
						SUBB		A,@R0            ;CRC��������յĶԱȣ�=0���
						JNZ     UC1              ;�д���ת��=0�����ж���һ�ֽ�
						INC			R0
						CLR			CY
						MOV			CRC0CN,#14H
						MOV			A,CRC0DAT
						SUBB		A,@R0
						JNZ			UC1
						SJMP		UC2
UC1:				CLR			RESIGN            ;CRC������֡������㣬���ݶ��������½���
						SETB		CRCER
						SJMP		UC3
UC2:	      CLR			CRCER
UC3:				NOP
						RET

        ;��ID�ţ�6�ֽڣ����浽65H��ʼ




			;����֡��������  �ӳ���			
DAPP:				NOP
						CLR			CY
						MOV			R0,#30H
						MOV			A,@R0
						CJNE		A,#55H,DW6       ;֡ͷ5555H�ж�
						INC			R0
						MOV			A,@R0
						CJNE		A,#55H,DW6
						MOV			R0,#37H           ;��Դ��ַ
						MOV			A,@R0
						CJNE		A,#1H,DW6
						MOV			R0,#3DH           ;��Ŀ���ַ�����ж��Ƿ����Լ�
						MOV			A,@R0
						CJNE		A,#00H,DAE2      ;=0�㲥ָ����봦�����ָ��
						SETB		BCSIGN
            SJMP		DAE1
DAE2:       CLR			BCSIGN
						CJNE		A,#05H,DW6       ;���Ǳ�����ַ�������������������
DAE1:			  ;CLR			BCSIGN
						INC			R0                 ;����ˮ�Ų�����
						MOV			A,@R0              
						JZ			DW1              ;=0,��ˮ������
						MOV			FNUM,A
						SJMP    DW2	
DW6:				LJMP		DAP1	
DW1:				MOV			FNUM,#0
DW2:				INC			R0                ;�����ֵ
						MOV			A,@R0
						CJNE		A,#05H,DW3       ;=05H  ���ص�ָ��
						SJMP		DW4
DW3:				CJNE		A,#03H,DW4      ;=03H ��ѯָ��
						NOP
						LJMP    PACK
DW4:				INC			R0               ;��ָ�����
						MOV			A,@R0
						CJNE		A,#00H,DAPN
					  SETB		P2.0		        ;�ص�
						SETB		P2.1
						MOV			PCA0CPL0,#0FFH  ;  =FFFFH,���=0V  
    				MOV			PCA0CPH0,#0FFH  ;  PCA0��Сռ�������д���ֽ�����
					  MOV   	PCA0CPL1,#0FFH  ;
    				MOV	  	PCA0CPH1,#0FFH  ;
            CLR			DATERR
						;CLR			BCSIGN
						SJMP		DQ4
DAPN:       CJNE		A,#03H,DQ2
						SJMP		DQ1             ;˫·��
DQ2:				CJNE		A,#02H,DQ3
						CLR			P2.1             ;��2·��
						INC			R0
						INC			R0
						MOV			A,@R0
						CPL			A
						MOV			PCA0CPL1,A
						MOV			PCA0CPH1,A
						CLR			DATERR
						SJMP		DQ4
DQ3:				CJNE		A,#01H,DAERR
						CLR			P2.0             ;��1·��
						LCALL		WAT
						INC			R0
						MOV			A,@R0
						CPL			A
						MOV			PCA0CPL0,A
						MOV			PCA0CPH0,A
						CLR			DATERR
						SJMP		DQ4
DQ1:        CLR			P2.0		        ;˫����
						CLR			P2.1
						LCALL		WAT              ;��ʱ
            INC			R0
						MOV			A,@R0            ;��һ·��������
						CPL			A
						MOV			PCA0CPL0,A;#00H  ;  =00H,���=10V  
    				MOV			PCA0CPH0,A;#00H  ;  PCA0��Сռ�������д���ֽ�����
						INC			R0               ;��2·��������
						MOV			A,@R0
						CPL			A
					  MOV   	PCA0CPL1,A;#00H  ;
    				MOV	  	PCA0CPH1,A;#00H  ;
						CLR			DATERR
						;CLR			BCSIGN
						SJMP		DQ4
DAERR:	    SETB		DATERR
DQ4:				NOP
						JB			BCSIGN,DAP1
						JB			DATERR,DQ5
						LCALL		AACK
						SJMP		DQ6
DQ5:				NOP			
						LCALL		NACK	
DQ6:				JNB			AUX,$
						NOP
						NOP
						NOP
						LCALL		USEND
						NOP
DAP1:				NOP	
						CLR			RESIGN
						CLR			FHFIND
						
						RET

		;�ط����ݰ���ǰ14�ֽ�����
FFRM:	;			NOP
			;			MOV		4BH,#55H
			;			MOV		4CH,#55H      ;֡ͷ
			;			MOV	  R0,#32H
			;			MOV		R1,#53H    ;��Դ��ַת�浽���͵�Ŀ�ĵ�ַ��6�ֽ�
			;			MOV		R7,#6
FF1:	;			MOV		A,@R0
			;			MOV		@R1,A
			;			INC		R0
			;			INC		R1
			;			DJNZ	R7,FF1
			;			MOV		R0,#38H
			;			MOV		R1,#4DH      ;��Ŀ�ĵ�ַת��Ϊ���͵�Դ��ַ
			;			MOV		R7,#6
FF2:	;			MOV		A,@R0
			;			MOV		@R1,A
			;			INC		R0
			;			INC		R1
			;			DJNZ	R7,FF2
			;			RET
	 
	 
	 ;PULL  ָ��ط�����  =03H
PACK:				NOP
						CLR			A
						MOV		 	DPTR,#TABS
						MOV			CRC0CN,#1DH
						MOV			R1,#4BH
						MOV			R7,#0EH
PCK1:				MOVC	 	A,@A+DPTR
						MOV			@R1,A
						INC			DPTR
						INC			R1	
						CLR			A
						DJNZ		R7,PCK1
					;	LCALL			FFRM
						MOV			A,FNUM           ;֡��ˮ+1
						INC			A   
						MOV			FNUM,A        
						MOV			@R1,A
						INC			R1
						MOV			@R1,#04H         ;���=4
						INC			R1                ;����ֵ=00
						MOV			@R1,40H
						INC			R1
						MOV			@R1,41H
						INC			R1
						MOV			@R1,42H
						INC			R1
						MOV			@R1,43H
			
						MOV			R7,#14H
						MOV			R1,#4BH
PCK2:				MOV			A,@R1
						MOV			CRC0IN,A
						INC			R1
						DJNZ		R7,PCK2
						MOV			A,CRC0DAT         ;���CRCֵ
						MOV			@R1,A
						MOV			CRC0CN,#14H
						INC			R1
						MOV			A,CRC0DAT
						MOV			@R1,A
						NOP
						JNB			AUX,$          ;AUX=1 ���Է�������
						NOP
						NOP
						NOP
						NOP
						NOP
						LCALL		USEND
						NOP
						CLR			RESIGN
						CLR			FHFIND
						MOV			R0,#30H
						SETB		EA
						LJMP		MAIN
				
						;��Ӧ��֡
AACK:				NOP
						CLR			A
						MOV		 	DPTR,#TABS
						MOV			CRC0CN,#1DH
						MOV			R1,#4BH
						MOV			R7,#0EH
ACK1:				MOVC	 	A,@A+DPTR
						MOV			@R1,A
						INC			DPTR
						INC			R1	
						CLR			A
						DJNZ		R7,ACK1
					;	LCALL			FFRM
						MOV			A,FNUM           ;֡��ˮ+1
						INC			A   
						MOV			FNUM,A        
						MOV			@R1,A
						INC			R1
						MOV			@R1,#01H         ;���=1
						INC			R1                ;����ֵ=00
						MOV			@R1,#00H
						INC			R1
						MOV			@R1,#00H
						INC			R1
						MOV			@R1,#00H
						INC			R1
						MOV			@R1,#00H
			
						MOV			R7,#14H
						MOV			R1,#4BH
						MOV			CRC0CN,#1DH
						MOV			CRC0CN,#1DH
LL:					MOV			A,@R1
						MOV			CRC0IN,A
						INC			R1
						DJNZ		R7,LL
						MOV			A,CRC0DAT         ;���CRCֵ
						MOV			@R1,A
						MOV			CRC0CN,#14H
						INC			R1
						MOV			A,CRC0DAT
						MOV			@R1,A
            RET
           
					  ;��Ӧ��
NACK:	      NOP
						NOP
						CLR			A
						MOV		 	DPTR,#TABS
						MOV			CRC0CN,#1DH
						MOV			R1,#4BH
						MOV			R7,#0EH
NCK1:				MOVC	 	A,@A+DPTR
						MOV			@R1,A
						MOV			CRC0IN,A
						INC			DPTR
						INC			R1	
						CLR			A
						DJNZ		R7,NCK1
					;	LCALL			FFRM
						MOV			A,FNUM           ;֡��ˮ+1
						INC			A   
						MOV			FNUM,A        
						MOV			@R1,A
						INC			R1
						MOV			@R1,#02H         ;���=2
						INC			R1                ;����ֵ=00
						MOV			@R1,#00H
						INC			R1
						MOV			@R1,#00H
						INC			R1
						MOV			@R1,#00H
						INC			R1
						MOV			@R1,#00H

						MOV			R7,#14H
						MOV			R1,#4BH
						MOV			CRC0CN,#1DH
LA:					MOV			A,@R1
						MOV			CRC0IN,A
						INC			R1
						DJNZ		R7,LA
						MOV			A,CRC0DAT         ;���CRCֵ
						MOV			@R1,A
						MOV			CRC0CN,#14H
						INC			R1
						MOV			A,CRC0DAT
						MOV			@R1,A
			
            RET
	
	
	 			;;;;�����ӳ���;;;;;					   
USEND:			NOP
						MOV			R1,#4BH
						MOV			R7,#16H
SED1:				MOV			A,@R1
						MOV     SBUF0, A        ;����
        		JNB     SCON0.1, $
						CLR     SCON0.1
						INC			R1
						DJNZ		R7,SED1
						NOP
						RET	
          
					 ;--------------------------------------------- 
  	WAT: MOV   R7, #0            ;��12mS�ӳ���
				 MOV   R6, #224
	 WAT1: NOP
	 			 DJNZ  R7, $
				 DJNZ  R6, WAT1          
         RET
        
				
	TABS:  DB		55H,55H,00H,00H,00H,00H,00H,05H
				 DB		00H,00H,00H,00H,00H,01H,04H,01H,00H,00H,00H,00H

END
