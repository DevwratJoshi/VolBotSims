import os

ends = [i for i in range(2, 10+1)]

retdir = os.getcwd()

for i in ends:
    os.chdir('data'+str(i))
    os.getcwd()
    for f in os.listdir():
        os.replace(f, f+'_40')
        
    os.chdir(retdir)


