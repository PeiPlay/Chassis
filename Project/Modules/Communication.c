#include "Communication.h"
#include "cmsis_os.h"
#define COMMUNICATION_ENDLESS_LOOP_BEGIN for(;;){
#define COMMUNICATION_ENDLESS_LOOP_END }

//得到当前Fifo中堆积的任务个数
uint32_t _Communication_GetFifoTaskNum(Communication_t* commu)
{
    return commu->taskFifo.task_cnt;
}
uint8_t _Communication_IsFifoEmpty(Communication_t* commu)
{
    return commu->taskFifo.task_cnt == 0;
}
uint8_t _Communication_IsFifoFull(Communication_t* commu)
{
    return commu->taskFifo.task_cnt == COMMUNICATION_TASK_FIFO_BUFSIZE;
}
void _Communication_FifoPush(Communication_t* commu, Communication_Task_t* task)
{
    if(_Communication_IsFifoFull(commu)) return;
    commu->taskFifo.buffer[commu->taskFifo.tail] = *task;
    commu->taskFifo.tail = (commu->taskFifo.tail + 1) % COMMUNICATION_TASK_FIFO_BUFSIZE;
    commu->taskFifo.task_cnt++;
}
void _Communication_FifoPop(Communication_t* commu, Communication_Task_t* task)
{
    if(_Communication_IsFifoEmpty(commu)) return;
    *task = commu->taskFifo.buffer[commu->taskFifo.head];
    commu->taskFifo.head = (commu->taskFifo.head + 1) % COMMUNICATION_TASK_FIFO_BUFSIZE;
    commu->taskFifo.task_cnt--;
}
void _Communication_FifoPopVoid(Communication_t* commu)
{
    if(_Communication_IsFifoEmpty(commu)) return;
    commu->taskFifo.head = (commu->taskFifo.head + 1) % COMMUNICATION_TASK_FIFO_BUFSIZE;
    commu->taskFifo.task_cnt--;
}
Communication_Task_t* _Communication_FifoFront(Communication_t* commu)
{
    if(_Communication_IsFifoEmpty(commu)) return NULL;
    return &commu->taskFifo.buffer[commu->taskFifo.head];
}
void _Communication_SendTask(Communication_t* commu)
{
    Communication_Task_t* task = _Communication_FifoFront(commu);
    FdcanBsp_Transmit_Classic_StdData(
        commu->hfdcan,
        ((uint32_t)task->receiverId << 8) | ((uint32_t)task->senderId << 4) | ((uint32_t)task->taskCode & 0xF),
        task->data,
        FDCAN_DLC_BYTES_8
    );
}
void _Communication_ReceiveTask(FdcanBspReceive_t* rxmsg, Communication_Task_t* rxtask)
{
    rxtask->dataLength = rxmsg->RxMessage.DataLength;
    rxtask->receiverId = (Communication_BoardID_t)((rxmsg->RxMessage.Identifier >> 8) & 0xF);
    rxtask->senderId = (Communication_BoardID_t)((rxmsg->RxMessage.Identifier >> 4) & 0xF);
    rxtask->taskCode = (uint8_t)(rxmsg->RxMessage.Identifier & 0xF);
    for(uint8_t i = 0; i < rxtask->dataLength; ++i)
        rxtask->data[i] = rxmsg->RxData[i];
}





/*
	外部接口的函数实现
*/
uint8_t Communication_AddTask(Communication_t* commu, Communication_Task_t* task)
{
    if(_Communication_IsFifoFull(commu)) return 0;
    _Communication_FifoPush(commu, task);
    return 1;
}
void Communication_Init(Communication_t* commu)
{
    commu->taskFifo = (Communication_TaskFifo_t){
        .head = 0,
        .tail = 0,
        .task_cnt = 0,
        .operate_cnt = 0,
        .offline_cnt = 0
    };
}
void Communication_TaskThread(Communication_t* commu)
{
    COMMUNICATION_ENDLESS_LOOP_BEGIN    
    osDelay(1);
    Communication_Task_t* task = _Communication_FifoFront(commu);
    if(task == NULL) continue;
    commu->taskFifo.operate_cnt++;
    if(task->isExecuted)
    {
        _Communication_FifoPopVoid(commu);
        commu->taskFifo.offline_cnt = 0;
        commu->taskFifo.operate_cnt = 0;
        continue;
    }
    else if(commu->taskFifo.offline_cnt < commu->offline_retry)
    {
        if(commu->taskFifo.operate_cnt % commu->operate_period == 0) {
            _Communication_SendTask(commu);
            commu->offline_retry++;
        }
    }
    else//超过重发次数，进行错误处理并丢弃任务
    {
        /*错误处理*/


        /*错误处理*/
        _Communication_FifoPopVoid(commu);
        commu->taskFifo.offline_cnt = 0;
        commu->taskFifo.operate_cnt = 0;
    }
    COMMUNICATION_ENDLESS_LOOP_END
}
__weak void Communication_FDCAN_Callback(FdcanBspReceive_t* rxmsg, Communication_t* commu)
{
    if( rxmsg->hfdcan               != commu->hfdcan        || 
        rxmsg->RxMessage.FDFormat   != FDCAN_CLASSIC_CAN    || 
        rxmsg->RxMessage.IdType     != FDCAN_STANDARD_ID    ||
        rxmsg->RxMessage.Identifier > 0x1FF                 ) return;
    Communication_Task_t rxtask; 
    _Communication_ReceiveTask(rxmsg, &rxtask);

    if( _Communication_FifoFront(commu)->senderId == rxtask.receiverId  && 
        _Communication_FifoFront(commu)->receiverId == rxtask.senderId  &&
        rxtask.taskCode == 0xF                                          )
    {   //收到对方的应答
        _Communication_FifoFront(commu)->isExecuted = 1;
    }
    else
    {  //收到对方的任务

    }
}


