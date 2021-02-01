#ifndef TORCALCONTROLLER_H
#define TORCALCONTROLLER_H

#include "controller.h"

/* 计算力矩法 */
class TorCalController : public Controller
{
public:
    TorCalController();
    ~TorCalController() {}
};

#endif // TORCALCONTROLLER_H
