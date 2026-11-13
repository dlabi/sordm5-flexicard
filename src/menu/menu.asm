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
COMMIT_ROM      EQU 13

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
BUFFER          EQU 7802H ;buffer for filename transfer
RST4JMP         EQU TO_RET
RST5JMP         EQU 0
AUTOSTART       EQU 2009h
FLAG            EQU 200Bh
HEAD:           EQU 200Ch
SIZE:           EQU 200Eh
STACK           EQU 07300h
ROM_LAUNCH_RAM  EQU 07E00h
DEBUG           EQU 9010h ; debug program e.g. from debug.cas

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
        call WAIT_IDLE
        jp c, directory_error
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
         call z, DEBUG          ;call debug
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
         push hl
         ld de,(NUMFILES)
         or a
         sbc hl,de
         pop hl
         jr c, valid_selection
         pop bc
         jp waitkey             ; empty slot on the final page
valid_selection:
         di
         CMD LOAD_FILE           ;destroys reg a
         ld c, DATA_PORT
         out (c),h              ;send higher byte
         DELAY 13
         out (c),l              ;send lower byte
         DELAY 13
         call WAIT_IDLE
         pop bc
         ei
         jr nc, load_result
         ld a,0ffh             ; distinguish timeout from FatFs errors
         jr report_error
load_result:
         in a,(DATA_PORT)       ; 0=OK, otherwise FatFs FRESULT
         or a
         jr z, fileready
report_error:
         push bc
         push af
         ld hl, loaderror 
         call DSPLTB
         pop af
         ld c,a
         call OUTHEX
         pop bc
         jp waitkey
fileready:
         ld a,(FLAG)
         cp 4
         jp z, launch_rom
         cp 1
         jp z, msx
         cp 2
         jp nz, waitkey

         ; Autostart machine code. Stack grows below the supported payload area.
         di
         ld sp,STACK
         ld hl,(AUTOSTART)
         push hl
         ld de,(HEAD)
         bit 7,d
         jr nz, cas_start       ; destination starts in expansion RAM
         CMD OFFSET_RAM_ON
         ld hl,1000h
         add hl,de              ; access the shadow of internal RAM at +1000h
         ld bc,(SIZE)
cas_copy:
         ld a,(hl)
         ld (de),a
         inc hl
         inc de
         dec bc
         ld a,b
         or c
         jr z, cas_copy_done
         bit 7,d
         jr z, cas_copy         ; stop at 8000h; remaining data is already mapped
cas_copy_done:
         CMD OFFSET_RAM_OFF
cas_start:
         pop hl
         ei
         jp (hl)

launch_rom:
         di
         ld hl,rom_launch_stub
         ld de,ROM_LAUNCH_RAM
         ld bc,rom_launch_end-rom_launch_stub
         ldir
         jp ROM_LAUNCH_RAM

; Relative branches remain valid after copying this code into internal RAM.
; COMMIT_ROM never accesses SD: it only copies the already validated image.
rom_launch_stub:
         ld sp,07DF0h
         ld a,COMMIT_ROM
         out (CMD_PORT),a
         DELAY 17
rom_commit_wait:
         in a,(CMD_PORT)
         bit ACTIVE,a
         jr nz,rom_commit_wait
         in a,(DATA_PORT)
         or a
         jp z,0000h             ; let the monitor initialize the new cartridge
         ei
         jp start               ; rejected commit leaves the old menu intact
rom_launch_end:

; Bounded wait for SD work. Carry reports a timeout; BREAK cancels the request.
WAIT_IDLE:
         push bc
         push de
         ld b,16
         ld de,0
wait_idle_poll:
         in a,(CMD_PORT)
         bit ACTIVE,a
         jr z,wait_idle_ready
         dec de
         ld a,d
         or e
         jr nz,wait_idle_poll
         djnz wait_idle_poll
         ld a,BREAK
         out (CMD_PORT),a
         scf
         jr wait_idle_exit
wait_idle_ready:
         or a
wait_idle_exit:
         pop de
         pop bc
         ret

directory_error:
         ld hl,loaderror
         call DSPLTB
         call ACECHI
         jp start

msx:    di
        ld sp,STACK
        ; Install the five-byte switch outside both BIOS and game payload.
        ld hl,msx_switch_stub
        ld de,(HEAD)
        ld bc,msx_switch_end-msx_switch_stub
        ldir
        ld hl,8000h
        ld de,7000h
        ld bc,1000h
        ldir
        ; The copied BIOS now owns 7000h-7fffh: no CALL/PUSH from here.
        ld a,OFFSET_RAM_OFF
        out (CMD_PORT),a
        ld hl,(AUTOSTART)
        ld (7cf1h),hl           ; original game autostart
        ld a,1
        ld (8000h),a            ; original BIOS RAM-probe sentinel
        ld hl,(HEAD)
        jp (hl)
msx_switch_stub:
        out (30h),a
        jp 0
msx_switch_end:

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

head:           db "FLEXI CARD MENU v0.19 A.Dlabac 13/11/26\r", 0
nofiles:        db "NO FILES FOUND :-(", 0
loaderror:      db "ERROR LOADING ROM ", 0
page:           db "PAGE ", 0
foot:           db " use <- or -> for page change", 0

IF ROM
        defs 6ff0h-$,0ffh
        defs 8,0
ENDIF
