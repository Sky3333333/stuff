num=1
Prims=[2]
while True:
    Prim=True
    num+=1
    for i in Prims:
        if num%i==0:
            Prim=False
    if Prim==True:
        Prims.append(num)
        print(num)