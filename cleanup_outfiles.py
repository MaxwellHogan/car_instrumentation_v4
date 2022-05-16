import os 

output_path = "output_files/"

c = str(input("Are you sure you want to clean files?  (Y/N) "))

if c.lower() != "y":
    raise ValueError('A very specific bad thing happened.')
 
## assumes that output_path only contains folders
for out_dir in os.listdir(output_path):
    print("cleaning: ", output_path+out_dir)
    os.system("rm -r {}".format(output_path+out_dir))
    os.system("mkdir {}".format(output_path+out_dir))
