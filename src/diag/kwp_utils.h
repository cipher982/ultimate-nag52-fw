
#ifndef __KWP_UTILS_H__
#define __KWP_UTILS_H__

#include "endpoints/endpoint.h"
#include "stdint.h"
#include "persistence_safety.h"

// Couple of helpful functions
void global_make_diag_neg_msg(DiagMessage *dest, uint8_t sid, uint8_t nrc);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, const uint8_t* resp, uint16_t len);

void global_make_diag_pos_msg(DiagMessage *dest, uint8_t sid, uint8_t pid, const uint8_t* resp, uint16_t len);


#endif