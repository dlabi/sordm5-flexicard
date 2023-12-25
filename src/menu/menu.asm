ACTIVE          EQU 7
BREAK           EQU 0
GET_COUNT       EQU 1
SET_INDEX       EQU 2
GET_INDEX       EQU 3 
NEXT_FILE       EQU 4
PREV_FILE       EQU 5
FIRST_FILE      EQU 6
GET_FILENAME    EQU 7
LOAD_FILE       EQU 8
RESET_SORD      EQU 9
DIR_SORD        EQU 10
OFFSET_RAM_ON   EQU 11
OFFSET_RAM_OFF  EQU 12

DATA_PORT       EQU 80H
CMD_PORT        EQU 81H

MAXLINES        EQU 18 ; lines on the page

TEXTMODE        EQU 0D04H
DSPLTB          EQU 105Bh ; prints ctrl characters
DSPLTA          EQU 105Ch ; executes ctrl characters
CTRLF           EQU 10EDh
SCNKB           EQU 0966H
ACECHI          EQU 0845H
PUTCHAR         EQU 1088H ;DSPCH
CLRSC           EQU 1393H ; clear screen
DECTR           EQU 090BH ;DECODEKEY ctr
DECAD           EQU 08DAH ; decodekey
CLKBF           EQU 077BH ; clears kb buffer
TO_RET          EQU 002EH; ret
BEL             EQU 1176H
MLTAL           EQU 1441H ; HL=A*L 	
IPL             EQU TO_RET
NUMFILES        EQU 7800H
BUFFER          EQU 7801H ;buffer for filename transfer
RST4JMP         EQU TO_RET
RST5JMP         EQU 0
AUTOSTART       EQU 2009h
FLAG            EQU 200Bh
HEAD:           EQU 200Ch
SIZE:           EQU 200Eh
STACK           EQU 0ff00h 

CMD     MACRO X
        ld a, X
        call cmd
        ENDM

DELAY   MACRO X
        rept X
        nop
        ENDM
        ENDM

;ROM EQU 0 ovladano z radky

IF ROM
        ORG 2000H
        DB 00           ; rom type
        DW BEGIN        ; rom start
        DW IPL          ; rom loader 
        DB 0xc3         ; 2005               
        DW RST4JMP      ; 2006 N/A
        DB 0xc3         ; 2008
        DW RST5JMP      ; 2009 autostart
flag:
        DB 00           ; 200B flag
        DW 0000         ; 200C header
        DW 0000         ; 200E length

ELSE
        ORG 7800H
ENDIF

BEGIN:
        call TEXTMODE
        call BEL
start:  CMD DIR_SORD
w1:     in a,(CMD_PORT)
        bit ACTIVE,a
        jr nz, w1               ; wait active goes down
        CMD GET_COUNT
        ;DELAY 20
        in a, (DATA_PORT)       ; higher byte
        ;DELAY 20
        ld b,a
        in a,(DATA_PORT)        ;lower byte
        ld c,a
        or b
        ld (NUMFILES), bc
        jr nz, firstpage
        ld hl, nofiles
        call DSPLTB
        jp waitkey
firstpage:
        ld ix, 0                ; page number
rstidx: ld iy, 0                ; index counter

newpage:
        push bc
        call HEADER             ; prints header
        ld a, ixl               ; page
        ld l, MAXLINES + 1
        call MLTAL              ; HL=L*A
        CMD SET_INDEX           ; set index of the first file on the page
w2:     in a,(CMD_PORT)
        bit ACTIVE,a
        jr z, w2                ; wait for active bit
        ld c, DATA_PORT         ; ready to set the index
        out (c),h
        DELAY 10
        out (c),l
        pop bc
next:   ld de, BUFFER
        call GETFILENAME
        ld hl, BUFFER
        push bc
        ld a, iyl
        call PRINT_INDEX
        call PRINT_FILENAME
        pop bc      
        dec bc
        ld a,iyl
        cp MAXLINES
        jr c, notyet
        call FOOTER
        jp waitkey
 notyet:
        ld a,b
        or c
        jr z, lastfile
        CMD NEXT_FILE
        in a, (DATA_PORT)
        inc iy
        jp next
 lastfile:       
        call FOOTER
        jp waitkey

NEXTPAGE:
         ld a, b
         or c
         jp z, waitkey          ; all files printed
         inc ix
         ld a, ixl
         or a
         CMD NEXT_FILE
         ;in a, (DATA_PORT)     ;neni treba, nepouziva se 
         jr z, over             ; overflow over 255
         jp rstidx
PREVPAGE:    
         ld a, ixl
         or a                   ; page 0?
         jp z, waitkey
         inc iy  
         add iy, bc             ; bc + iy + MAXLINES + 1
         ld b,0
         ld c, MAXLINES + 1
         add iy,bc
         push iy
         pop bc
over:    dec ix
         jp rstidx


cmd:    out (CMD_PORT), a
        DELAY 17
        ret

GETFILENAME:
        ; IN:   DE BUFFER
        ; OUT:  HL LENGTH OF DATA READ
        ;ld de, BUFFER

        push de

        ld a, GET_FILENAME
        out (CMD_PORT), a
        DELAY 30
loop:   ;in a,(CMD_PORT)
        ;DELAY 30
        ;bit ACTIVE,a
        ;jr z, done
        ;DELAY 20
        in a,(DATA_PORT)
        ld (de),a
        or a
        jr z, done
        inc de
        DELAY 20
        jr loop

done:   pop de
        ex de, hl
        sbc hl,de
        ret

PRINT_INDEX:
        add a, 41h              ;0=A, 1=B and so on
        call PUTCHAR
        ld a, " "
        call PUTCHAR
        ret

PRINT_FILENAME:   
        push hl
        push de
        call DSPLTB
        call CTRLF
        sbc hl,de
        pop de
        pop hl
        jr c, PRINT_FILENAME
endd:   ret

waitkey: push bc
         call ACECHI            ;wait for key
         push af
         call CLKBF             ;clear kb buffer
         pop af
         pop bc
         cp 0DH
         call z, 9010h          ;call debug
         cp 3AH
         jp z, NEXTPAGE
         cp 3BH
         jp z, PREVPAGE
         cp 061H
         jp  C, start
         cp 061H + MAXLINES + 1
         jp nc, start           ; out of range start again
index:   sub 61H
         push bc
         push af
         ld a, ixl
         ld l, MAXLINES + 1
         call MLTAL              ; HL=L*A
         pop af
         ld b,0
         ld c,a
         add hl, bc             ; page * maxlines + index
         di
         CMD LOAD_FILE           ;nici reg a
         ld c, DATA_PORT
         out (c),h              ;send higher byte
         DELAY 13
         out (c),l              ;send lower byte
         DELAY 13
w:       in a,(CMD_PORT)
         cp LOAD_FILE or 80H    ; are we ready?
         jr z, w
         pop bc
         ei
         DELAY 13    
         in a,(DATA_PORT)       ; load status 0 = OK FF=KO
         or a
         jp z, fileready
         ld hl, loaderror 
         call DSPLTB
         jp waitkey
fileready:
         ld a, (FLAG)         ; read extra status
         cp 1                 ; MSX
         jr z, msx
         cp 2                 ; cas with autostart loaded 
         jr nz, nextcheck
         ld hl, HEAD
         ld a, (HL)
         bit 7, A
         jr z, need_copy        ;program starts in internal ram, needs to be copied there first
         rst 28h
nextcheck: cp 3
         jp z, basic        
         jp waitkey

need_copy:
        CMD OFFSET_RAM_ON
        di
        LD SP,STACK
        ld hl,8300h
        ld de,7300h
        ld bc,1000h - 300h
        ;ld hl,HEAD+1000h
        ;ld de,HEAD
        ;ld bc,SIZE
        ;bit 7,d
        ;jr nz, done1              ;we are outside of internal ram 
        ldir
 done1:
        ei
        CMD OFFSET_RAM_OFF
        ;call 9010h
        rst 28h

msx:    di
        LD SP,STACK         
        ld hl,8000h
        ld de,7000h
        ld bc,1000h
        ldir
        CMD OFFSET_RAM_OFF      ; zrusi offset
        ld hl,(AUTOSTART)
        ld (7cf1h), hl ; patchni msx aby po startu skocilo do hry
        ld a,1
        ld (8000h),a ; snad pomuze k neprepisovani 1. byte hry
        out (30h),a     ; disable roms
        rst 0

basic:
        di
        ld hl,(HEAD)
        ld bc, 1000h
        add hl,bc
        ld de,(HEAD)
        ld bc,(SIZE)
        push de
        push bc
        ldir
        CMD OFFSET_RAM_OFF ; disable offset 
        pop bc
        pop de
        ld (726ah), de
        ld ix, (HEAD)
        add ix, bc
        ld (726ch),ix
        ld (726eh),ix
        ld a, 2            ; disable Cartridge, MONITOR keeps     
        out (30h),a
        rst 0


OUTHEX:
; Input: c
   ld  a,c
   rra
   rra
   rra
   rra
   call CONV
   ld  a,c
CONV:
   and  $0F
   add  a,$90
   daa
   adc  a,$40
   daa
   call PUTCHAR
   ret

DELAY2:
        PUSH DE
        PUSH AF
Outer:
        LD DE, 1000h            ;Loads DE with hex 1000
Inner:
        DEC DE                  ;Decrements DE
        LD A, D                 ;Copies D into A
        OR E                    ;Bitwise OR of E with A (now, A = D | E)
        JP NZ, Inner            ;Jumps back to Inner: label if A is not zero
        DEC BC                  ;Decrements BC
        LD A, B                 ;Copies B into A
        OR C                    ;Bitwise OR of C with A (now, A = B | C)
        JP NZ, Outer            ;Jumps back to Outer: label if A is not zero
        pop af
        POP DE
        RET                     ;Return from call to this subroutine


HEADER:
        call CLRSC
        ld hl, head
        call DSPLTB
        call CTRLF
        ret

FOOTER:
        push hl
        push de
        push bc
        call CTRLF
        ld hl, page
        call DSPLTB
        ld a, ixl
        add a, 30h
        call PUTCHAR
        ld hl, foot
        call DSPLTB
        pop bc
        pop de
        pop hl
        ret

head:           db "FLEXI CARD MENU v0.7 Ales Dlabac 12/25\r", 0
nofiles:        db "NO FILES FOUND :-(", 0
loaderror:      db "ERROR LOADING ROM", 0
page:           db "PAGE ", 0
foot:           db " use <- or -> for page change", 0


