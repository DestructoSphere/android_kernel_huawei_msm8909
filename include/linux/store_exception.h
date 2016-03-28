#ifndef __LINUX_STOREEXCEPTION_H
#define __LINUX_STOREEXCEPTION_H

/**
 *  name: the name of this command
 *  msg: concrete command string to write to /dev/log/exception
 *  return: on success return the bytes writed successfully, on error return -1
 *
*/
int store_exception(char* name, char* msg);


#endif
