#include "ryhandlib.h"
#include <stdio.h>

// 定义一个函数指针类型，它匹配我们希望从Python传入的回调函数签名
typedef void(*PythonCallback_t)(u8_t*, void*);

// 全局变量，用于存储从Python传入的回调函数指针
static PythonCallback_t g_python_callback = NULL;

// C "蹦床" 函数。这个函数将被传递给AddListen。
// 它的签名与C库期望的完全匹配。
void trampoline_callback(CanMsg_t stuMsg, void * para)
{
    if (g_python_callback != NULL)
    {
        // 当被C库调用时，它会调用存储的Python回调函数指针，
        // 并将复杂的CanMsg结构体简化为最基本的u8_t*指针。
        g_python_callback(stuMsg.pucDat, para);
    }
}

// 一个由Python调用的设置函数，用于将Python回调函数的地址存储在C的全局变量中
void RyRegisterCallback(PythonCallback_t callback)
{
    g_python_callback = callback;
}

