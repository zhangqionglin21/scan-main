#include "export.h"
#include "algorithm.h"

ALGORITHM_LIB_API IAlgorithm* createInterface()
{
    return Algorithm::createInstance();
}