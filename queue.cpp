#include "queue.h"
#include "state.h"
#include "planner_math.h"

static constexpr size_t MOVE_QUEUE_SIZE = 32;
static LineMove moveQueue[MOVE_QUEUE_SIZE];
static volatile uint8_t queueHead=0, queueTail=0, queueCount=0;

bool is_queue_empty(){ return queueCount==0; }
bool is_queue_full(){ return queueCount>=MOVE_QUEUE_SIZE; }
uint8_t get_queue_count(){ return queueCount; }
void clear_queue(){ queueHead=queueTail=queueCount=0; }

bool queue_move(const LineMove& move_in){
  if(is_queue_full()) return false;
  LineMove m = move_in;

  if(queueCount>0){
    uint8_t idx_prev = (uint8_t)((queueTail + MOVE_QUEUE_SIZE - 1) % MOVE_QUEUE_SIZE);
    LineMove &prev = moveQueue[idx_prev];

    int32_t sx0,sy0,sz0;
    if(queueCount>=2){
      uint8_t idx_prev2 = (uint8_t)((queueTail + MOVE_QUEUE_SIZE - 2) % MOVE_QUEUE_SIZE);
      sx0 = moveQueue[idx_prev2].sx; sy0 = moveQueue[idx_prev2].sy; sz0 = moveQueue[idx_prev2].sz;
    }else{
      sx0 = realtime_X; sy0 = realtime_Y; sz0 = realtime_Z;
    }

    float pvx = (float)(prev.sx - sx0), pvy = (float)(prev.sy - sy0), pvz = (float)(prev.sz - sz0);
    float nvx = (float)(m.sx - prev.sx), nvy = (float)(m.sy - prev.sy), nvz = (float)(m.sz - prev.sz);

    float theta = angle_between_vec(pvx,pvy,pvz, nvx,nvy,nvz);

    float v_prev_cruise = fminf(MAX_FEED_STEPS_S, 1e6f/(float)prev.d_cruise_us);
    float v_new_cruise  = fminf(MAX_FEED_STEPS_S, 1e6f/(float)m.d_cruise_us);

    float v_corner = corner_speed_limit(theta);
    v_corner = fminf(v_corner, fminf(v_prev_cruise, v_new_cruise));
    v_corner = fmaxf(v_corner, MIN_CORNER_SPEED);

    uint32_t d_corner_us = period_us_from_speed(v_corner);
    prev.d_end_us = d_corner_us;
    m.d_start_us  = d_corner_us;

    moveQueue[idx_prev] = prev;
  }

  moveQueue[queueTail] = m;
  queueTail = (uint8_t)((queueTail + 1) % MOVE_QUEUE_SIZE);
  queueCount++;
  return true;
}

bool dequeue_move(LineMove* m){
  if(is_queue_empty()) return false;
  *m = moveQueue[queueHead];
  queueHead = (uint8_t)((queueHead + 1) % MOVE_QUEUE_SIZE);
  queueCount--;
  return true;
}
