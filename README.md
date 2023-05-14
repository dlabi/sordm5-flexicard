# sordm5-flexicard

If no ROM is found on SD card, flexicard emulates 64kbf ver 2C RAM cartridge. See modes below



            Verze LZR ( 2C )
            ================
            
            +------------+
            |////////////|	READ ONLY
            +------------+
            |\\\\\\\\\\\\|	WRITE ONLY
            +------------+
            |XXXXXXXXXXXX|	RW
            +------------+
            |            |	DISABLED
            +------------+
            
            	0   0   0   1   1   2   2   2   3   3   4   4   4   5   5   6   6
            kB	0   4   8   2   6   0   4   8   2   6   0   4   8   2   6   0   4
            	+-------+-------------------+
            ROM	|MONITOR|      BASIC-F      |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
            RAM	|       |       |       |       |       |       |       |       |
            	+-------+-------+-------+-------+-------+-------+-------+-------+
        CART    |       |       |       |       |       |       |       |       |
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|///////|///////////////////|
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 0	|       |       |       |   |XXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|       |                   |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 1	|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|///////|                   |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 2	|\\\\\\\|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|       |                   |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 3	|///////|///////|///////|///|XXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|       |                   |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 4	|///////|///////|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|///////|///////////////////|
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 5	|\\\\\\\|\\\\\\\|\\\\\\\|\\\|XXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            
            	+-------+-------------------+
            	|///////|                   |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 6	|\\\\\\\|\\\\\\\|\\\\\\\|\\\|XXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
						|///////|///////|///|
						+-------+-------+---+
            
            	+-------+-------------------+
            	|       |                   |
            	+-------+-------+-------+---+---+-------+-------+-------+-------+
        MOD 7	|       |       |       |       |       |       |       |       |
            	+-------+-------+-------+-------+-------+-------+-------+-------+
            	|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|XXXXXXX|
            	+---------------------------------------------------------------+
