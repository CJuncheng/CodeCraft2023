# 输入到 stdout 
puts("hello world") // 单纯打印字符串
printf("hello world %d %d\n", a, b)  
fprintf(stdout, "hello world %d %d\n", a, b)

# 输入到 stdin
scandf("%d", &a)

# 输入到stderr
fprintf(stderr,"hello world %d %d\n", a, b);
perror("hello world") // 单纯打印字符串
