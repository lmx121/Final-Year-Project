/*
*********************************************************************************************************
*
*	模块名称 : GUI任务
*	文件名称 : lvgl_task.h
*	版    本 : V1.0
*	说    明 : 
*********************************************************************************************************
*/
#ifndef __LVGL_TASK_H
#define	__LVGL_TASK_H

#ifdef __cplusplus
extern "C" {
#endif


#define I2C_ADDR_FT6336 0x38

#define TP_PRESS_DOWN    0x80  //触摸屏被按下，0x10000000,第七位为1
#define TP_COORD_UD      0x40  //触摸屏坐标更新，第六位为1



extern void lvgl_task(void * parm);
extern void touch_task(void *arg);
extern void scan_xpt2046(void);


#ifdef __cplusplus
}
#endif


#endif