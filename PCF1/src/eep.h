
extern int eep_init(void);
extern int eep_test(short adr, short val) ;
extern int set_eep_char(short adr, char val) ;
extern int set_eep_short(short adr, short val) ;
extern int set_eep_int(short adr, int val);
extern unsigned char get_eep_char(short adr);
extern short get_eep_short(short adr);
extern int get_int (short adr);
extern int set_eep_block(short adr, short * val, short len );
extern int get_eep_block(short adr, short * val, short len );  