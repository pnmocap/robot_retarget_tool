#include "MotionAdaptorLib.h"
#include "FscAlgorithmImpl.h"

namespace eba
{

	FscAlgorithm* CreateFscAlgorithm()
	{
		return new FscAlgorithmImpl();
	}

	void DestroyFscAlgorithm(FscAlgorithm* fsc)
	{

	}

}

