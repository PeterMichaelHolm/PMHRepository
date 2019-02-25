extern void eep_get_block( int adr,char * block);
extern void eep_set_block( int adr,char * block);
extern void init_eep(void);
extern int get_eep_int(int block,int adr);
//extern int set_eep_int(int block,int adr,int val);
extern char eep_get_status(void);
extern int get_eep_id(void );  
extern int setup_eep( void);
extern int test_eep(void);
// löschen eines Blocks im Flash = 8 pages = 2 KB 
extern int del_bereich(int ab, int anzahl);
// löschen eines Sektors im Flash = 128 pages = 64 KB 
extern int del_sektoren(int ab, int anzahl);
// Speichern eines ram Bereiches , egal wie groß dieser ist
// der speicherbereich muß aber vorher mit int del_bereich(int ab, int anzahl)
// gelöscht werden
extern int store_struct(char * struktur, int adress, int len);
// laden eines flash Bereiches in das ram
extern int get_struct(char * struktur, int adresse, int len);

extern int find_offset(void);
extern int store_offset(int val);


//#define eep_adr_start_prozess1    8
