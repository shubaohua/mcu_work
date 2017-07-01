;                 智能路灯终端节点主程序 ，C8051F410，11.0592MHz    2017.05.10  LHB
             
						  ;硬件版本 SMART-L-T01 V1.2B
             ;*************************************************
             ;检测电能数据，SPI总线扩展芯片扩展2个串口，串口A用电能检测，串口B环境数据检测模块
						 ;通过无线接收上报数据，串口
             ;输出2路PWM信号，上拉到5V，频率500Hz
						 ;模拟输出由PWM滤波产生，1-10V，提高输出频率
             ;输出2路继电器
						 ;I2C总线挂 EEPROM，保存ID号
						 ;P1.7   AUX
						 ;30H开始为接收数据
						 ;4B开始为发送帧
						;2016-6-14 帧长改为22字节
						 ;65H开始6字节为本机ID号，开机后从EEPROM读
						 ;增加广播数据标志，广播时不回发
							;2017-7-1 增加开机通电
             ;*************************************************
						

						AUX	 		BIT		P1.7       ;无线模块状态检测，=1空闲
						RESIGN	EQU   22H.0      ;接受帧满标记=1，字节满可处理
						CRCER   EQU   22H.1      ;CRC计算接收的对比错误标记，=1，有错，要发负应答
   					DATERR  EQU		22H.2      ;数据解析时有误，=1有错，要发负应答
						FNUM		EQU   7BH        ;帧流水号
 						FHFIND	EQU		22H.5	
						BCSIGN	EQU		22H.6      ;广播标志，=1为广播，不发数据

         $INCLUDE (C8051F410.inc)
            CSEG  AT 0
						SJMP  PROG
						ORG   0023H
						LJMP	URT0       ;串口中断入口
            ORG   0030H
      PROG: MOV   PCA0MD, #00H      ;410初始化。disable watchdog timer
           ;用外部晶振               内部晶振频率的时钟为24.5MHZ/128
            MOV   OSCXCN, #67H      ;外部晶体用11.0592MHZ, 内部振荡器为24.5/128(MHZ)
						MOV   R7, #70           ;wait 1ms
            DJNZ  R7, $
    OSCwai: MOV   A, OSCXCN         ;查询 XTLVLD-->1
            JNB   ACC.7, OSCwai
            NOP					  
						NOP
						;I/O口设置
            MOV   P0MDIN, #0FCh     ;P0不是模拟输入
            MOV   P0MDOUT, #0FFH     ;P0.7 推挽输出
            MOV   P1MDIN, #0FCh     ;P1.0,P1.1为模拟输入，P1.2-P1.7为数字输入
            MOV   P1MDOUT, #0FCh     ;P1.2~P1.7 推挽输出
   	        MOV   P2MDIN, #0FFh     ;P2 口为数字输入
            MOV   P2MDOUT, #127     ;P2.0-P2.6 推挽输出
   	        MOV   XBR0, #07h        ;UART,SPI,IIC,CEX0,CEX1接到端口
            MOV   P1SKIP, #03h      ;P1.0-P1.1 跳过
						MOV   P0SKIP,#03H       ;P0.0,P0.1  d/a跳过
            MOV   XBR1, #42h        ;交叉开关使能 
            MOV   CLKSEL, #31H      ;系统时钟用外部时钟11.0592MHZ。
						MOV		CLKMUL,#02H
						;MOV		OSCICN,#86H

            ;参考电压设置
						MOV   REF0CN,#18H     ;vdd做参考，输出2.25v

           ;定时器和串口配置
            MOV   TCON,#10H
						MOV   TMOD, #22H        ;T0,T1 8位自动重装,T0用于IIC，T1用于串口波特率
						MOV   CKCON, #5         ;T0,T1用系统时钟，用于波特率设置						
						MOV   TL0, #0A4H        ;IIC=40k
            MOV   TH0, #0A4H
						MOV	  SMB0CF,#80H;
            MOV   TH1, #70H         ;UART0波特率 9600b/S,spi=6400
            MOV   TL1, #70H
						
						;SPI  配置（串口扩展）
						MOV   SPI0CFG,#40H;      4线主机模式
    				MOV   SPI0CN,#05H;
    				MOV		SPI0CKR,#0B7H;      SCK=30K
						
						;DAC   左对齐，写高字节启动,满幅0.5mA
						MOV   IDA0CN,#0F1H
						MOV   IDA1CN,#0F1H
						NOP
						MOV		IDA0L,#0
						MOV		IDA1L,#0
						MOV		IDA0H,#80H      ;中间幅度输出
						MOV		IDA1H,#80H 
						NOP     ;

						;PWM 设置
						MOV		PCA0CN,#40h       ;使用T0溢出率做时钟（40K）,周期6.4ms
						ANL   PCA0MD,#0BFH
					;	MOV		PCA0MD,#04H
						MOV	  PCA0MD,#0       ;关看门狗，PCA时钟=系统时钟/12 ，模拟输出时用
    				MOV		PCA0CPM0,#42H  ;     8位PWM输出
    				MOV		PCA0CPM1,#42H  ;
    				MOV		PCA0L,#80H
						MOV		PCA0H,#80H
						NOP
    				MOV		PCA0CPL0,#0FFH  ;  =FFFFH,输出=0V  
    				MOV		PCA0CPH0,#0FFH  ;  PCA0最小占空输出，写高字节启动
					  MOV   PCA0CPL1,#0FFH  ;
    				MOV	  PCA0CPH1,#0FFH  ;
						NOP

						;CRC32初始化
						MOV		CRC0CN,#1DH       ;CRC16,初始=FF,结果CRC0DAT从高字节开始读						 
					           
            LCALL WAT               ;
     BEGIN: MOV   SP, #0EFH         ;init stack pointer
            MOV	  R0, #255	        ;RAM清空
	          CLR   A
     CLEAR: MOV   @R0, A
            DJNZ  R0, CLEAR
						NOP
						MOV		R0,#30H           ;接收缓冲区开始地址
            SETB  TR0 
            SETB  TR1               ;UART0 工作
            MOV   SCON0, #10H       ;8位UART，接收使能
						MOV   IE,#90H           ;串口开中断
						SETB	P2.0           ;继电器0关
						SETB  P2.1           ;继电器1关

 						; 串口扩展芯片初始化
		;				MOV		SPI0DAT,#18H    ;串1（电能）= 4800bps，8N1；串2（环境）=9600，8N1
		;				MOV		SPI0DAT,#80H    ;向LCR写80H
		;				MOV		SPI0DAT,#00H
		;				MOV		SPI0DAT,#18H    ;向DLL写18H    4800bps
		;				MOV		SPI0DAT,#01H
		;				MOV		SPI0DAT,#00H    ;向DLH写0H
		;				MOV		SPI0DAT,#18H
		;				MOV		SPI0DAT,#0BBH   ;向LCR写BBH，8N1
		;				MOV		SPI0DAT,#18H
		;				MOV		SPI0DAT,#03H		;向LCR写03H
		;				MOV		SPI0DAT,#08H
		;				MOV		SPI0DAT,#01H    ;IER写01H，开接收中断
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
						CLR			RESIGN    ;22H.0        ;RESIGN   帧接收标记
						CLR			CRCER     ;22H.1          ;CRCER    CRC错误标记
						CLR			DATERR    ;22H.2        ;DATERR   数据解析错误标记
						CLR			22H.5
						CLR			22H.6
						MOV			FNUM,#0
						CLR			BCSIGN
						SETB		EA
						NOP
						CLR			P2.0
						NOP	
						CLR			P2.1
						MOV			PCA0CPL0,#0H  ;  =00H,输出=10V  
    				MOV			PCA0CPH0,#0H  ;  PCA0最大占空输出，写高字节启动
					  MOV  	  PCA0CPL1,#0H  ;
    				MOV	 	  PCA0CPH1,#0H  ;
						NOP

          ;主程序  
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
						

					

      ;串口接收中断处理
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
						SETB		RESIGN             ;收满22字节，=1 关中断不再继续接收
						MOV			R0,#30H
						CLR			EA
						SJMP		UU2
						NOP

UU1:				SETB		EA
UU2:				POP     ACC
						NOP
						RETI

   ;==================子程序区===================
           ;---------------------------------------------
           
			  ;计算 接收 CRC16
CRCD:				MOV		CRC0CN,#1DH
						MOV			R7,#14H          ;计算前20字节
						MOV			R0,#30H
UC4:				MOV			A,@R0
						MOV			CRC0IN,A
						INC			R0
						DJNZ		R7,UC4
						NOP	    
						MOV			R0,#44H
						CLR			CY
						MOV			A,CRC0DAT        ;CRC结果
						SUBB		A,@R0            ;CRC计算与接收的对比，=0相等
						JNZ     UC1              ;有错，跳转。=0继续判断下一字节
						INC			R0
						CLR			CY
						MOV			CRC0CN,#14H
						MOV			A,CRC0DAT
						SUBB		A,@R0
						JNZ			UC1
						SJMP		UC2
UC1:				CLR			RESIGN            ;CRC错，接收帧标记清零，数据丢弃，重新接收
						SETB		CRCER
						SJMP		UC3
UC2:	      CLR			CRCER
UC3:				NOP
						RET

        ;读ID号，6字节，保存到65H开始




			;数据帧解析处理  子程序			
DAPP:				NOP
						CLR			CY
						MOV			R0,#30H
						MOV			A,@R0
						CJNE		A,#55H,DW6       ;帧头5555H判断
						INC			R0
						MOV			A,@R0
						CJNE		A,#55H,DW6
						MOV			R0,#37H           ;读源地址
						MOV			A,@R0
						CJNE		A,#1H,DW6
						MOV			R0,#3DH           ;读目标地址，并判断是否是自己
						MOV			A,@R0
						CJNE		A,#00H,DAE2      ;=0广播指令，必须处理后续指令
						SETB		BCSIGN
            SJMP		DAE1
DAE2:       CLR			BCSIGN
						CJNE		A,#05H,DW6       ;不是本机地址，跳出，否则继续处理
DAE1:			  ;CLR			BCSIGN
						INC			R0                 ;读流水号并保存
						MOV			A,@R0              
						JZ			DW1              ;=0,流水号清零
						MOV			FNUM,A
						SJMP    DW2	
DW6:				LJMP		DAP1	
DW1:				MOV			FNUM,#0
DW2:				INC			R0                ;读标记值
						MOV			A,@R0
						CJNE		A,#05H,DW3       ;=05H  开关灯指令
						SJMP		DW4
DW3:				CJNE		A,#03H,DW4      ;=03H 查询指令
						NOP
						LJMP    PACK
DW4:				INC			R0               ;读指令参数
						MOV			A,@R0
						CJNE		A,#00H,DAPN
					  SETB		P2.0		        ;关灯
						SETB		P2.1
						MOV			PCA0CPL0,#0FFH  ;  =FFFFH,输出=0V  
    				MOV			PCA0CPH0,#0FFH  ;  PCA0最小占空输出，写高字节启动
					  MOV   	PCA0CPL1,#0FFH  ;
    				MOV	  	PCA0CPH1,#0FFH  ;
            CLR			DATERR
						;CLR			BCSIGN
						SJMP		DQ4
DAPN:       CJNE		A,#03H,DQ2
						SJMP		DQ1             ;双路开
DQ2:				CJNE		A,#02H,DQ3
						CLR			P2.1             ;第2路开
						INC			R0
						INC			R0
						MOV			A,@R0
						CPL			A
						MOV			PCA0CPL1,A
						MOV			PCA0CPH1,A
						CLR			DATERR
						SJMP		DQ4
DQ3:				CJNE		A,#01H,DAERR
						CLR			P2.0             ;第1路开
						LCALL		WAT
						INC			R0
						MOV			A,@R0
						CPL			A
						MOV			PCA0CPL0,A
						MOV			PCA0CPH0,A
						CLR			DATERR
						SJMP		DQ4
DQ1:        CLR			P2.0		        ;双开灯
						CLR			P2.1
						LCALL		WAT              ;延时
            INC			R0
						MOV			A,@R0            ;第一路调光数据
						CPL			A
						MOV			PCA0CPL0,A;#00H  ;  =00H,输出=10V  
    				MOV			PCA0CPH0,A;#00H  ;  PCA0最小占空输出，写高字节启动
						INC			R0               ;第2路调光数据
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

		;回发数据包的前14字节生成
FFRM:	;			NOP
			;			MOV		4BH,#55H
			;			MOV		4CH,#55H      ;帧头
			;			MOV	  R0,#32H
			;			MOV		R1,#53H    ;将源地址转存到发送的目的地址，6字节
			;			MOV		R7,#6
FF1:	;			MOV		A,@R0
			;			MOV		@R1,A
			;			INC		R0
			;			INC		R1
			;			DJNZ	R7,FF1
			;			MOV		R0,#38H
			;			MOV		R1,#4DH      ;将目的地址转存为发送的源地址
			;			MOV		R7,#6
FF2:	;			MOV		A,@R0
			;			MOV		@R1,A
			;			INC		R0
			;			INC		R1
			;			DJNZ	R7,FF2
			;			RET
	 
	 
	 ;PULL  指令回发处理  =03H
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
						MOV			A,FNUM           ;帧流水+1
						INC			A   
						MOV			FNUM,A        
						MOV			@R1,A
						INC			R1
						MOV			@R1,#04H         ;标记=4
						INC			R1                ;参数值=00
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
						MOV			A,CRC0DAT         ;获得CRC值
						MOV			@R1,A
						MOV			CRC0CN,#14H
						INC			R1
						MOV			A,CRC0DAT
						MOV			@R1,A
						NOP
						JNB			AUX,$          ;AUX=1 可以发送数据
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
				
						;正应答帧
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
						MOV			A,FNUM           ;帧流水+1
						INC			A   
						MOV			FNUM,A        
						MOV			@R1,A
						INC			R1
						MOV			@R1,#01H         ;标记=1
						INC			R1                ;参数值=00
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
						MOV			A,CRC0DAT         ;获得CRC值
						MOV			@R1,A
						MOV			CRC0CN,#14H
						INC			R1
						MOV			A,CRC0DAT
						MOV			@R1,A
            RET
           
					  ;负应答
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
						MOV			A,FNUM           ;帧流水+1
						INC			A   
						MOV			FNUM,A        
						MOV			@R1,A
						INC			R1
						MOV			@R1,#02H         ;标记=2
						INC			R1                ;参数值=00
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
						MOV			A,CRC0DAT         ;获得CRC值
						MOV			@R1,A
						MOV			CRC0CN,#14H
						INC			R1
						MOV			A,CRC0DAT
						MOV			@R1,A
			
            RET
	
	
	 			;;;;发送子程序;;;;;					   
USEND:			NOP
						MOV			R1,#4BH
						MOV			R7,#16H
SED1:				MOV			A,@R1
						MOV     SBUF0, A        ;发送
        		JNB     SCON0.1, $
						CLR     SCON0.1
						INC			R1
						DJNZ		R7,SED1
						NOP
						RET	
          
					 ;--------------------------------------------- 
  	WAT: MOV   R7, #0            ;等12mS子程序
				 MOV   R6, #224
	 WAT1: NOP
	 			 DJNZ  R7, $
				 DJNZ  R6, WAT1          
         RET
        
				
	TABS:  DB		55H,55H,00H,00H,00H,00H,00H,05H
				 DB		00H,00H,00H,00H,00H,01H,04H,01H,00H,00H,00H,00H

END
