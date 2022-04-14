#include "hello.h"
#include <gflags/gflags.h>

// 利用 gflag 提供的宏定义参数该宏的 3 个参数分别为命令行参数名，参数默认值，参数的帮助信息
DEFINE_int64(print_times, 2, "print_times(>=1)");


int main(int argc, char** argv ) {
	google::ParseCommandLineFlags(&argc, &argv, true);

	for(int i=0; i++; i<=FLAGS_print_times){
		sayHello();
 	}

}
