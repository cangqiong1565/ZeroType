#ifndef CHASE_LIGHT_OS_LIST_H
#define CHASE_LIGHT_OS_LIST_H

#include "portMacro.h"

//普通节点结构体
struct xLIST_ITEM
{
    //uint32_t
    TickType_t xItemValue;            //用于帮助节点做顺序排列（说明节点为什么当前处在链表的这个位置）
    struct xLIST_ITEM* pxNext;        //指向下一个节点
    struct xLIST_ITEM* pxPrevious;    //指向前一个节点
    void* pvOwner;                    //指向TCB
    void* pvContainer;                //指向这个节点所在的链表
};
typedef struct xLIST_ITEM ListItem_t;

//精节点结构体定义（用作标志链表的“队尾”）
struct xMINI_LIST_ITEM
{
    TickType_t xItemValue;            //用于帮助节点做顺序排列

    struct xLIST_ITEM* pxNext;        //指向下一个节点

    struct xLIST_ITEM* pxPrevious;    //指向前一个节点
};
typedef struct xMINI_LIST_ITEM MiniListItem_t;

//根节点结构体（链表表头/链表的管理器）
typedef struct xLIST
{
    UBaseType_t uxNumberOfItems; //链表节点计数器

    ListItem_t* pxIndex;         //链表节点索引指针

    MiniListItem_t xListEnd;     //链表最后一个节点
}List_t;

/* 初始化节点的拥有者（建立链表与TCB的联系）*/
#define listSET_LIST_ITEM_OWNER( pxListItem, pxOwner )\
( ( pxListItem )->pvOwner = ( void * ) ( pxOwner ) )

/* 获取节点拥有者（可以获取任意节点所属的TCB）*/
#define listGET_LIST_ITEM_OWNER( pxListItem )\
( ( pxListItem )->pvOwner )

/*
 * 获取链表第一个节点的拥有者 (即 TCB)
 * 原理：
 * 1. pxList->xListEnd 是链表的尾部哨兵节点。
 * 2. 哨兵节点的下一个 (pxNext) 就是链表的“头部”节点（第一个真实节点）。
 * 3. 获取那个节点的 pvOwner，就是我们要找的任务 TCB。
 */
#define listGET_OWNER_OF_HEAD_ENTRY( pxList )  ( (&( ( pxList )->xListEnd ))->pxNext->pvOwner )

/* 初始化节点排序辅助值 （将任意节点排序辅助值初始化为我设定的那个值）*/
#define listSET_LIST_ITEM_VALUE( pxListItem, xValue )\
( ( pxListItem )->xItemValue = ( xValue ) )

/* 获取节点排序辅助值 （可以获取到任意一个节点的排序辅助值）*/
#define listGET_LIST_ITEM_VALUE( pxListItem )\
( ( pxListItem )->xItemValue )

/* 获取链表中第一个节点的排序辅助值 */
#define listGET_ITEM_VALUE_OF_HEAD_ENTRY( pxList )\
( ( ( pxList )->xListEnd ).pxNext->xItemValue )

/* 获取链表的入口节点 */
#define listGET_HEAD_ENTRY( pxList )\
( ( ( pxList )->xListEnd ).pxNext )

/* 获取节点的下一个节点 */
#define listGET_NEXT( pxListItem )\
( ( pxListItem )->pxNext )

/* 获取链表的最后一个节点 */
#define listGET_END_MARKER( pxList )\
( ( ListItem_t const * ) ( &( ( pxList )->xListEnd ) ) )

/* 判断链表是否为空 */
#define listLIST_IS_EMPTY( pxList )\
( ( BaseType_t ) ( ( pxList )->uxNumberOfItems == ( UBaseType_t ) 0 ) )

/* 获取链表的节点数 */
#define listCURRENT_LIST_LENGTH( pxList )\
( ( pxList )->uxNumberOfItems )

/* * 核心功能：获取【下一个】获得 CPU 使用权的 TCB
 * * 逻辑：
 * 1. 让 pxIndex（遍历游标）向后移一位。
 * 2. 如果移到了 xListEnd（链表尾部/哨兵），则再移一位（跳回头部）。
 * 3. 取出当前指向节点的 pvOwner（TCB）。
 * * 作用：实现同优先级任务的时间片轮转调度 (Round Robin)。
 */
#define listGET_OWNER_OF_NEXT_ENTRY( pxTCB, pxList ) \
{ \
List_t * const pxConstList = ( pxList ); \
/* 节点索引指向链表第一个节点 */ \
( pxConstList )->pxIndex = ( pxConstList )->pxIndex->pxNext; \
if( ( void * ) ( pxConstList )->pxIndex == ( void * ) &( ( pxConstList )->xListEnd ) ) \
{ \
( pxConstList )->pxIndex = ( pxConstList )->pxIndex->pxNext; \
} \
/* 获取节点的 OWNER，即 TCB */\
( pxTCB ) = ( pxConstList )->pxIndex->pvOwner; \
}

/* 返回节点所在的链表（容器）指针 */
#define listLIST_ITEM_CONTAINER( pxListItem ) ( ( pxListItem )->pvContainer )

void vListInitialiseItem (ListItem_t * pxItem);
//根节点初始化
void vListInitialise( List_t * pxList );
//链表尾插
void vListInsertEnd (List_t * pxList, ListItem_t* pxNewListItem);
//按照升序排列插入链表
void vListInsert(List_t * pxList, ListItem_t* pxNewListItem);
//将节点从链表删除
UBaseType_t uxListRemove(ListItem_t * pxItemToRemove);


#endif //CHASE_LIGHT_OS_LIST_H