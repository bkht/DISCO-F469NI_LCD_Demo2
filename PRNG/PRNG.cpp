#include "PRNG.h"

PRNG::PRNG()
{
	m_z = 12434;
	m_w = 33254;
}

unsigned int PRNG::Rnd()
{
	m_z = 36969 * (m_z & 65535) + (m_z >>16);
	m_w = 18000 * (m_w & 65535) + (m_w >>16);
	return ((m_z <<16) + m_w);
}
