int exercise_1(const char *s){
	int res = 0;
	asm volatile (
		"mov r1, #0\n"
		"ldrb r2, [r0]\n"
		"loop: cbz r2, done\n"
		"add r1, #1\n"
		"add r0, #1\n"
		"ldrb r2, [r0]\n"
		"b loop\n"
		"done:\n"
		"mov %[res], r1\n"
		:[res] "=r" (res)
		:
		:
	);
	return res;
}
