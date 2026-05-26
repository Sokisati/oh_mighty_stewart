import os
with open('stewart_final_benchmark.m', 'r', encoding='utf-8') as f:
    code = f.read()
code = code.replace("''", "'")
with open('stewart_final_benchmark.m', 'w', encoding='utf-8') as f:
    f.write(code)
print('Done!')
