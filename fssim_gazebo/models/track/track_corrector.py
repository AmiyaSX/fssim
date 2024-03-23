import sys

# path = 'ejemplo.sdf'
path = sys.argv[1]
print(path)

left = 0
right = 0
orange = 0

circuit = []
with open(path, 'r') as fr:
    circuit = fr.readlines()
    
fw = open(path, 'w') 
for line in circuit:
    new_line = line
    if '<name>' in line:
        r = ''
        s = ''
        if 'right' in line:
            r = 'cone_right'
            s = r + str(right)
            right += 1
        elif 'left' in line:
            r = 'cone_left'
            s = r + str(left)
            left += 1
        elif 'orange' in line:
            r = 'cone_orange'
            s = r + str(orange)
            orange += 1
        new_line = line.replace(r, s)
    fw.write(new_line)

if left>0 and right>0 and orange>0:
    print('Circuito corregido')
