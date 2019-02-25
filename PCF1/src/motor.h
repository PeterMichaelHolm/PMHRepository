

extern int set_motor_power_on(void) ;
extern int set_motor_power_off(void);
extern int set_motor_entlage_links(void);
extern int set_motor_entlage_rechts(void);
extern int set_motor_stop(void);
extern int set_motor_startpos(void);
extern int auto_setup(void);
extern int probefahrt(int zeit); 


extern void set_motor_drehzahl(float drehzahl,float steps,float min_drehzahl,int array);
extern void set_motor_drehzahlr(float drehzahl,float steps,float min_drehzahl, int array);
extern void set_motor_timer(void); 
extern void set_motor_start(void); 
extern void set_motor_stope(void);
extern void set_motor_stoppen(void);
extern int get_motor_state(void); 
extern void set_motor_default(void);
extern void set_motor_timer(void);  