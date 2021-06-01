#include "pcv.h"
#include "LexyCore.h"
#include <frameio.h>
#include <time.h>
#include <stdio.h>

int main() 
{
	/// 准备数据
	Reprojector rj;

	//std::vector<std::string> full_path = { "./LexyData/AZ00.ptcloud" ,"./LexyData/AZ01.ptcloud" ,"./LexyData/AZ02.ptcloud","./LexyData/AZ03.ptcloud" };
	//std::vector<std::string> full_path = { "./LexyData/AB.ptcloud" ,"./LexyData/AB00.ptcloud" ,"./LexyData/AB01.ptcloud" ,"./LexyData/AB02.ptcloud" };
	//std::vector<std::string> full_path = { "./LexyData/LM05.ptcloud" ,"./LexyData/LM02.ptcloud" ,"./LexyData/LM03.ptcloud" ,"./LexyData/LM04.ptcloud" };
	std::vector<std::string> full_path = { "./LexyData/data1.ptcloud" ,"./LexyData/data2.ptcloud" ,"./LexyData/data3.ptcloud" ,"./LexyData/data4.ptcloud" };
	//std::vector<std::string> full_path = { "./LexyData/NZ01.ptcloud" ,"./LexyData/NZ02.ptcloud" ,"./LexyData/NZ03.ptcloud" ,"./LexyData/NZ04.ptcloud" };
	//std::vector<std::string> full_path = { "./LexyData/SPD01.ptcloud" ,"./LexyData/SPD02.ptcloud" ,"./LexyData/SPD03.ptcloud" ,
		//"./LexyData/SPD04.ptcloud","./LexyData/SPD05.ptcloud","./LexyData/SPD06.ptcloud","./LexyData/SPD07.ptcloud","./LexyData/SPD08.ptcloud" };

	/// 1.初始化
	LaxyCore lac;
	lac.init();
	lac.setParam("enable_debug",0);
	/// 2.载入模板
	
	//std::string template_name = "AZ2000_1223384_A";
	//std::string template_name = "AZ2000_1223384_B";
	//std::string template_name = "LM-C3806D_C3806-13";
	std::string template_name = "select";
	//std::string template_name = "NZ805UR_1283465";
	//std::string template_name = "SPD601_680";
	if (!lac.reload_template(template_name)) return 0;

	for (int i = 0; i < full_path.size(); i++)
	{
		PtCloud container;
		container = rj.loadPTData(full_path[i]);

		/// 3.输入带测量数据，返回结果
		std::vector<LaxyRes> res;

		auto start = clock();
		lac.generate(container, res);
		
		auto cost = clock() - start;
		std::cout << "MATCH Time (ms): " << (double)cost / CLOCKS_PER_SEC * 1000 << std::endl;
		///3D可视化（可选）
		lac.vistual();
	}
	
	system("pause");

	return 1;

}