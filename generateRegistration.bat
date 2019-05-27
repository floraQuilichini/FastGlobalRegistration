
SET PATH=C:\FPFH\generateFPFH_files\generateFPFH_files
SET NAME_SOURCE=C:\FPFH\generateFPFH_files\generateFPFH_files\ObjetSynthetique_simp_
SET SOURCE1_PROP=_down_m0_s0.33333XZ
SET SOURCE2_PROP=_down_m0_s0.66667XZ
SET SOURCE3_PROP=_down_m0_s1XZ
SET EXT_SOURCE=.bin
SET NAME_OUTPUT=C:\FastGlobalRegistration.git\FastGlobalRegistration\output
SET EXT_OUTPUT=.txt

echo %SOURCE1_PROP%
SET OUT_NAME=%NAME_OUTPUT%%EXT_OUTPUT%
echo %OUT_NAME%


for %%t in (C:\FPFH\generateFPFH_files\generateFPFH_files\ObjetSynthetique_clean_full_res_m0_s0.16667.bin, C:\FPFH\generateFPFH_files\generateFPFH_files\ObjetSynthetique_clean_full_res_m0_s0.33333.bin, C:\FPFH\generateFPFH_files\generateFPFH_files\ObjetSynthetique_clean_full_res_m0_s0.5.bin) do (
  for %%m in (2, 4, 8, 16, 32, 64) do (
	SET FULL_SOURCE1_NAME=%NAME_SOURCE%%%m%SOURCE1_PROP%%EXT_SOURCE%
	echo %FULL_SOURCE1_NAME%
	SET FULL_OUTPUT1_NAME=%NAME_OUTPUT%%%m%SOURCE1_PROP%%EXT_OUTPUT%
	echo %FULL_OUTPUT1_NAME%
    SET FULL_SOURCE2_NAME=%NAME_SOURCE%%%m%SOURCE2_PROP%%EXT_SOURCE%
	echo %FULL_SOURCE2_NAME%
    SET FULL_OUTPUT2_NAME=%NAME_OUTPUT%%%m%SOURCE2_PROP%%EXT_OUTPUT%
	echo %FULL_OUTPUT2_NAME%
    SET FULL_SOURCE3_NAME=%NAME_SOURCE%%%m%SOURCE3_PROP%%EXT_SOURCE%
	echo %FULL_SOURCE3_NAME%
    SET FULL_OUTPUT3_NAME=%NAME_OUTPUT%%%m%SOURCE3_PROP%%EXT_OUTPUT%
	echo %FULL_OUTPUT3_NAME%
    C:\FastGlobalRegistration-build\FastGlobalRegistration\Release\FastGlobalRegistration.exe %FULL_SOURCE1_NAME% %%t %FULL_OUTPUT1_NAME% 
    C:\FastGlobalRegistration-build\FastGlobalRegistration\Release\FastGlobalRegistration.exe %FULL_SOURCE2_NAME% %%t %FULL_OUTPUT2_NAME%
    C:\FastGlobalRegistration-build\FastGlobalRegistration\Release\FastGlobalRegistration.exe %FULL_SOURCE3_NAME% %%t %FULL_OUTPUT3_NAME%
	)
  
  )

