#include "mbed.h"

#ifndef __PRNG_H__
#define __PRNG_H__

class PRNG {
public:
PRNG();
unsigned int Rnd();

private:
unsigned int m_z, m_w;

protected:

};

#endif
