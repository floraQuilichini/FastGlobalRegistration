import subprocess
import os

executable = "C:\FastGlobalRegistration-build\FastGlobalRegistration\Release\FastGlobalRegistration.exe"
output_name_prefix = "C:\FastGlobalRegistration.git\FastGlobalRegistration\output"
output_ext = ".txt"
input_name_target_prefix = "C:\FPFH\generateFPFH_files\generateFPFH_files\ObjetSynthetique_clean_full_res_"
input_ext = ".bin"
input_name_source_prefix = "C:\FPFH\generateFPFH_files\generateFPFH_files\ObjetSynthetique_simp_"


for target_prop in ["m0_s0.16667", "m0_s0.33333", "m0_s0.5"]:
	input_target_file = input_name_target_prefix + target_prop + input_ext

	for simp_level in ["2", "4", "8", "16", "32", "64"]:
	
		for source_prop in ["m0_s0.33333XZ", "m0_s0.66667XZ", "m0_s1XZ"]:
	
			input_source_file = input_name_source_prefix + simp_level + "_down_" + source_prop + input_ext
			output_file = output_name_prefix + simp_level + source_prop + target_prop + output_ext
			
			args = executable + " " + input_source_file + " " + input_target_file + " " + output_file
			subprocess.call(args, stdin=None, stdout=None, stderr=None)
	