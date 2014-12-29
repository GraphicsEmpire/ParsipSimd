#include "PS_SketchConfig.h"

namespace PS{
	CSketchConfig::CSketchConfig()
	{
		m_strFileName = "MEM";
		m_fmode = fmMemoryStream;
	}

	CSketchConfig::CSketchConfig( const std::vector<DAnsiStr>& inputContent )
	{
		m_strFileName = "MEM";
		m_fmode = fmMemoryStream;
		setContentBuffer(inputContent);
	}

	bool CSketchConfig::readIntArray(DAnsiStr section, DAnsiStr variable, int ctExpected, std::vector<int>& arrayInt)
	{
		DAnsiStr strVal;
		if(readValue(section, variable, strVal))
		{
			size_t pos;
			int iComp = 0;
			DAnsiStr strTemp;
			if(strVal.firstChar() == '(')
				strVal = strVal.substr(1);
			else
				return false;
			while(strVal.lfind(',', pos))
			{
				strTemp = strVal.substr(0, pos);
				strVal = strVal.substr(pos + 1);
				strVal.removeStartEndSpaces();
				arrayInt.push_back(atoi(strTemp.ptr()));
				iComp++;
			}

			if(strVal.length() >= 1)
			{
				if(strVal.lastChar() == ')')
				{
					strTemp = strVal.substr(0, strVal.length() - 1);
					strTemp.removeStartEndSpaces();
					arrayInt.push_back(atoi(strTemp.ptr()));
				}
			}
		}
		return ((int)arrayInt.size() == ctExpected);
	}

	int CSketchConfig::writeIntArray(DAnsiStr section, DAnsiStr variable, const std::vector<int>& arrayInt)
	{
		DAnsiStr strValue, strTemp;
		if(arrayInt.size() > 1)
		{
			for(size_t i=0; i<arrayInt.size(); i++)
			{
				if(i == 0)
					strTemp = printToAStr("(%d, ", arrayInt[i]);
				else if(i == arrayInt.size() - 1)
					strTemp = printToAStr("%d)", arrayInt[i]);
				else
					strTemp = printToAStr("%d, ", arrayInt[i]);
				strValue += strTemp;
			}
			writeValue(section, variable, strValue);
		}
		else if(arrayInt.size() == 1)
		{
			strTemp = printToAStr("(%d)", arrayInt[0]);
			writeValue(section, variable, strTemp);
		}
		else
			writeValue(section, variable, DAnsiStr("()"));
		return arrayInt.size();
	}
}