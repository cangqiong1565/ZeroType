#include "../Inc/List.h"
//链表初始化
//ListItem_t有五个成员，但是只将pvContainer指向空指针就可以了，表示这个链表为空
void vListInitialiseItem (ListItem_t * const pxItem)
{
    pxItem->pvContainer = NULL;
}

//根节点初始化
void vListInitialise( List_t * const pxList )
{
    //将索引指针指向最后一个节点
    pxList->pxIndex = (ListItem_t*) &(pxList->xListEnd);
    //将链表最后一个节点的辅助排序的值设置为最大，确保该节点就是链表的最后节点
    pxList->xListEnd.xItemValue = portMAX_DELAY;
    //将最后一个节点的前后指针都指向最后一个节点（也就是自身），表示链表为空
    pxList->xListEnd.pxNext = (ListItem_t*)&(pxList->xListEnd);
    pxList->xListEnd.pxPrevious = (ListItem_t*)&(pxList->xListEnd);
    //将节点计数器记为0
    pxList->uxNumberOfItems = (UBaseType_t) 0U;
}

//链表尾插
void vListInsertEnd (List_t * const pxList, ListItem_t * const pxNewListItem)
{
    ListItem_t * const pxIndex = pxList->pxIndex;

    pxNewListItem->pxNext = pxIndex;
    pxNewListItem->pxPrevious = pxIndex->pxPrevious;
    pxIndex->pxPrevious->pxNext = pxNewListItem;
    pxIndex->pxPrevious = pxNewListItem;

    //节点指向自己的链表（标注自己是哪个链表的）
    pxNewListItem ->pvContainer = (void*)pxList;

    //节点计数器++
    (pxList->uxNumberOfItems++);
}

//按照升序排列插入链表
void vListInsert(List_t * const pxList, ListItem_t * const pxNewListItem)
{
    ListItem_t *pxIterator;
    const TickType_t xValueOfInsertion = pxNewListItem->xItemValue;

    if (xValueOfInsertion == portMAX_DELAY)
    {
        pxIterator = pxList->xListEnd.pxPrevious;
    }
    else
    {
        for (   pxIterator = (ListItem_t*)&(pxList->xListEnd);
                pxIterator->pxNext->xItemValue <= xValueOfInsertion;
                pxIterator = pxIterator->pxNext)
        {
            //什么都不做，只为了循环找到指定位置
        }
    }

    pxNewListItem->pxNext = pxIterator->pxNext;
    pxIterator->pxNext->pxPrevious = pxNewListItem;
    pxNewListItem->pxPrevious = pxIterator;
    pxIterator->pxNext = pxNewListItem;

    pxNewListItem->pvContainer = (void*)pxList;
    ( pxList->uxNumberOfItems )++;
}


//将节点从链表删除
UBaseType_t uxListRemove(ListItem_t * const pxItemToRemove)
{
    //获取待删除节点属于那个链表
    List_t* const pxList = (List_t*)pxItemToRemove->pvContainer;

    //物理隔绝待删除节点
    pxItemToRemove->pxNext->pxPrevious = pxItemToRemove->pxPrevious;
    pxItemToRemove->pxPrevious->pxNext = pxItemToRemove->pxNext;

    //如果链表的遍历游标(pxIndex)正指向当前节点，则需要将游标回退一步
    if (pxList -> pxIndex ==  pxItemToRemove)
    {
        pxList->pxIndex = pxItemToRemove->pxPrevious;
    }

    //更新这个节点所属链表为空，并更新节点计数器
    pxItemToRemove->pvContainer = NULL;
    (pxList->uxNumberOfItems)--;

    //返回剩余节点个数
    return pxList->uxNumberOfItems;
}
