#pragma once
#include "types.h"

bool is_queue_empty();
bool is_queue_full();
bool queue_move(const LineMove& m);
bool dequeue_move(LineMove* m);
void clear_queue();
uint8_t get_queue_count();
