#pragma once
#ifndef PS_SKETCHCONFIG_H
#define PS_SKETCHCONFIG_H

#include <vector>
#include "PS_AppConfig.h"

using namespace std;

namespace PS{

	class CSketchConfig : public CAppConfig
	{
	public:
		CSketchConfig();

		CSketchConfig(const std::vector<DAnsiStr>& inputContent);

		CSketchConfig(DAnsiStr strFileName, FileMode mode)
		{
			set(strFileName, mode);
		};

		~CSketchConfig()
		{

		}

		bool readIntArray(DAnsiStr section, DAnsiStr variable, int ctExpected, std::vector<int>& arrayInt);
		int	writeIntArray(DAnsiStr section, DAnsiStr variable, const std::vector<int>& arrayInt);
	};


}
#endif