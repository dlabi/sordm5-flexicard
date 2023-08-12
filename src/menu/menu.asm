ACTIVE          EQU 7
BREAK           EQU 0
GET_COUNT       EQU 1
SET_INDEX       EQU 2
GET_INDEX       EQU 3 
NEXT_FILE       EQU 4
PREV_FILE       EQU 5
FIRST_FILE      EQU 6
GET_FILENAME    EQU 7
LOAD_ROM        EQU 8
RESET_SORD      EQU 9
DIR_SORD        EQU 10

DATA_PORT       EQU 80H
CMD_PORT        EQU 81H

TEXTMODE        EQU 0D04H
DSPLTB          EQU 105Bh 
CTRLF           EQU 10edh
SCNKB           EQU 0966H
PUTCHAR         EQU 1088H ;DSPCH
CLRSC           EQU 1393H ; clear screen
DECTR           EQU 090BH ;DECODEKEY 
TO_RET          EQU 002EH; ret
IPL             EQU TO_RET
BUFFER          EQU 7800H ;vyrovnavajici pamet pro prenos nazvu


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
        DB 00           ;rom type
        DW BEGIN        ;rom start
        DW IPL          ;rom loader
        ;DW RST4JMP
        ;DW RST5JMP

ELSE
        ORG 7800H
ENDIF

BEGIN:
        call TEXTMODE
start:  CMD DIR_SORD
        call CLRSC
        ld hl, text1
        call DSPLTB
        call CTRLF
        CMD GET_COUNT
        in a, (DATA_PORT) ; higher byte
        ld b,a
        in a,(DATA_PORT) ;lower byte
        ld c,a
        or b
        ;dec c
        jr nz, go
        ld hl, text2
        call DSPLTB
        jp waitkey
go:     CMD FIRST_FILE
next:   ld de, BUFFER
        call GETFILENAME
        ld hl, BUFFER
        push bc
        CMD NEXT_FILE
        in a, (DATA_PORT)
        DELAY 20
        ld c,a
        call OUTHEX
        ld a, " "
        call PUTCHAR
        call tisk
        pop bc      
        dec bc
        ld a,b
        or c
        jr nz, next
        jp waitkey


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


tisk:   push hl
        push de
        call DSPLTB
        call CTRLF
        sbc hl,de
        pop de
        pop hl
        jr c, tisk
endd:   ret

waitkey: call SCNKB
         or a
         jr z, waitkey
         call DECTR
         cp 0DH
         call z, 8010h
CISLO:   cp 030H
         jp  C, start 
         cp 040H
         jp NC, start
         sub 31H
         ld c, a
         di
         CMD LOAD_ROM           ;nici reg a
         ld a, c
         ld c, DATA_PORT
         ld b,0
         out (c),b              ;send higher byte
         DELAY 13
         out (c),a              ;send lower byte
         DELAY 13
w:       in a,(CMD_PORT)
         cp LOAD_ROM or 80H
         jr z, w
         ei
         DELAY 13       
         in a,(DATA_PORT)       ; load status 0 = OK FF=KO
         or a
         jp z, 0    ; reset sorda
         ld hl, text3 
         call DSPLTB
         jp waitkey

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



text1: db "FLEXI CARD MENU v0.2 Ales Dlabac 08/12", 0
text2: db "NO FILES FOUND :-(", 0
text3: db "ERROR LOADING ROM", 0

