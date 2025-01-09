# Java学习笔记

## 一、Java的诞生

​    Java programming language具有大部分编程语言所共有的一些特征，被特意设计用于[互联网](https://baike.baidu.com/item/互联网/199186?fromModule=lemma_inlink)的分布式环境。Java具有类似于C++语言的"形式和感觉"，但它要比C++语言更易于使用，而且在编程时彻底采用了一种"以对象为导向"的方式。使用Java编写的[应用程序](https://baike.baidu.com/item/应用程序/5985445?fromModule=lemma_inlink)，既可以在一台单独的电脑上运行，也可以被分布在一个网络的[服务器端](https://baike.baidu.com/item/服务器端/3369401?fromModule=lemma_inlink)和客户端运行。另外，Java还可以被用来编写容量很小的[应用程序](https://baike.baidu.com/item/应用程序/5985445?fromModule=lemma_inlink)模块或者applet，做为[网页](https://baike.baidu.com/item/网页/99347?fromModule=lemma_inlink)的一部分使用。applet可使[网页](https://baike.baidu.com/item/网页/99347?fromModule=lemma_inlink)使用者和[网页](https://baike.baidu.com/item/网页/99347?fromModule=lemma_inlink)之间进行交互式操作。

Java是Sun微系统公司在1995年推出的，推出之后马上给[互联网](https://baike.baidu.com/item/互联网/199186?fromModule=lemma_inlink)的交互式应用带来了新面貌。最常用的两种互联网[浏览器](https://baike.baidu.com/item/浏览器?fromModule=lemma_inlink)软件中都包括一个Java[虚拟机](https://baike.baidu.com/item/虚拟机?fromModule=lemma_inlink)。几乎所有的操作系统中都增添了Java[编译程序](https://baike.baidu.com/item/编译程序/8290180?fromModule=lemma_inlink)。

## 二、常用Dos命令

>  盘符切换 F:回车  或者 cd /d F:
>  查看当前路径下目录结构 dir
>  当前目录下路径切换 cd 文件夹名  cd .. 返回上一级
>  清屏 cls （clear screen）
>  退出终端 exit
>  查看电脑IP ipconfig
>  文件夹操作 
>  创建文件夹 md dirname
>  删除文件夹 rd dirname
>  在当前路径下创建文件 cd>test.txt
>  删除文件 del test.txt
>  打开应用
>  calc 计算器
>  notepad 记事本

## 三、基本数据类型

<img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230224155157188.png" alt="image-20230224155157188" style="zoom:50%;" />

### 2.1 类型转换

> byte short int 在做数值计算时会自动转换为int

> 占用字节数多的向低的转换会丢失精度

### 2.2 转义字符

<img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230224223133067.png" alt="image-20230224223133067" style="zoom:50%;" />

### 2.3运算符

- 算数运算符

<img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230225112437886.png" alt="image-20230225112437886" style="zoom:50%;" />

- 逻辑运算符

<img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230225123540672.png" alt="image-20230225123540672" style="zoom:50%;" />

- 位运算符

1. 按位取反~
2. 按位与&
3. 按位或|
4. 按位异或^

- 三目运算符

> X ? Y : Z   , 条件X是否成立，成立则执行Y，不成立则执行Z

### 2.4 分支语句

```java
if(条件语句){
    // 执行体
}else{
    // 执行体
}
```



```java
if(条件语句){
    // 执行体
}else if{
    // 执行体
}else if{
    // 执行体
}
...
 else{

}
```

```java
// 获取用户输入
System.out.println("请输入年份");
Scanner input = new Scanner(System.in);
int year = input.nextInt();
```

```java
switch(表达式){           // 表达式支持int byte char short string
    case 取值1:语句块1;
        break;
    case 取值2:语句块2;
        break;
    case 取值n:语句块n;
        break;
    default:语句块n+1；   // 之前的case都未匹配，则执行
        break;
}
```

### 2.5 循环语句

```java
while(条件表达式){
    // 语句块;
}
```

```java
do{
    // 语句块;
}while();
```

> 前两者的区别在于前一个可能一次都不执行，后一个至少执行一次

```java
for(初始化参数;判断条件;更新循环变量){
    //循环体;
}
//关键字：continue表示跳过当此循环，继续下次循环；break表示直接跳出循环
//注：判断条件省略，死循环
```

## 四、方法与数组

### 3.1形参和实参

> 在方法定义时设置的参数称为形参
>
> 调用方法时传递的参数称为实参

### 3.2 方法重载

> 在类中可以创建多个方法，他们具有相同的名字，但具有不同的参数和不同的定义。

> 注意：返回值不能作为方法重载的条件

> 方法重载的要求是两同一不同：同一个类中方法名相同，参数列表不同。至于方法的其他部分，如方法返回值类型、修饰符等，与方法重载没有任何关系。

```java
public class Hello{
    public static void main(String[] args){
        
    }
    public static void printMessage(String message){
        System.out.println(message);
    }
    public static void printMessage(char message){
        System.out.println(message);
    }
}
```

### 3.3 数组

>  一组能够存储相同数据类型的的数据集合

```java
// 四种定义方法
// 1
int[] score = new int[5];
// 2
int[] score;
score = new int[5];
// 3
int[] score = new int[]{1,2,3,4,5};
// 4
int[] score = {1,2,3,4,5}

// 数组的长度
System.out.println(score.length);
```

> 数组的遍历
>
> ```java
> int[] score = {1,2,3,4,5};
> // foreach
> for(int i : score){
>     System.out.println(i);
> }
> ```
>
> 
>
> 可变参数
>
> ```java
> public class Hello{
> 	public static void main(String[] args){
> 		int[] score = {1,2,3,4,5};
> 		
> 		getMenu(1,2,3,4,5);
> 	}
> 	public static void getMenu(int k,int... menu){
> 		System.out.println(k);    // k输出是1；
> 		for(int i : menu){
> 			System.out.println(i);
> 		}
> 	}	
> }
> ```
>
> 
>
> 注意
>
> - 不能直接打印数组
> - 当一个变量为null时，去调用该变量的属性和方法，会报空指针异常，NullPointException
>
> 

> 多维数组
>
> Java中没有真正的多维数组，多维数组的表示方式是数组中的元素还是数组

> Arrays类
>
> - 常用方法：
>
> | 1    | boolean equals(int[] a,int[] b)    | 判断两个数组是否相等               |
> | ---- | ---------------------------------- | ---------------------------------- |
> | 2    | String toString(int[] a)           | 输出数组信息                       |
> | 3    | void fill(int[] a, int val)        | 将指定值填充到数组之中             |
> | 4    | void sort(int[] a)                 | 对数组进行排序                     |
> | 5    | int binarySearch(int[] a, int key) | 对排序后的数组进行二分法检索指定值 |
>
> - 代码示例：
>
> ```java
> package com.hy.contact;
> 
> import java.util.Arrays;
> 
> public class ArraysTest {
> 	/**
> 	 * @param args
> 	 */
> 	public static void main(String[] args) {
> 		//boolean equals(int[] a,int[] b)
> 		int[] arr1 = new int[] {1,2,3,4,5};
> 		int[] arr2 = new int[] {1,3,2,4,5};
> 		boolean isEquals = Arrays.equals(arr1,arr2);  // 注意：两个数组相等元素顺序也要相等
> 		System.out.println(isEquals);
> 		
> 		//String toString(int[] a)
> 		System.out.println(Arrays.toString(arr1));    // 输出[1, 2, 3, 4, 5]
> 
> 		//void fill(int[] a, int val)
> 		Arrays.fill(arr1,10);
> 		System.out.println(Arrays.toString(arr1));    // 输出[10, 10, 10, 10, 10]
> 
> 		//void sort(int[] a)
> 		Arrays.sort(arr2);
> 		System.out.println(Arrays.toString(arr2));
> 		//int binarySearch(int[] a, int key)
> 		
> 		int a = Arrays.binarySearch(arr2, 10);   // 返回索引，从0开始，未找到返回负数
> 		if(a > 0) {
> 			System.out.println(a);
> 		}else {
> 			System.out.println("未找到");
> 		}
> 		
> 		
> 	}
> }
> 
> ```

> 数组异常
>
> - 数组角标越界异常：ArrayIndexOutOfBoundsExcetion
> - 空指针异常：NullPointerException ,
>
> ```java
> package com.hy.contact;
> 
> public class ArraysException {
>     public static void main(String[] args) {
>     	// 数组空指针异常的情况
>     	//1
> //    	int[] arr1 = new int[] {1,2,3};
> //    	arr1 = null;
> //    	System.out.println(arr1[0]);
>     	
>     	//2
> //    	int[][] arr2 = new int[4][];
> //    	System.out.println(arr2[0][0]);
>     	
>     	//3
>     	String[] arr3 = new String[] {"AA","BB","CC"};
>     	arr3[0] = null;
>     	System.out.println(arr3[0].toString());
>     }
> }
> 
> ```
>
> 

## 五、Eclipse的使用

> 初始启动配置
>
> 1.修改字符集，window-preferences
>
> ![image-20230301205733849](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230301205733849.png)
>
> 2.修改界面,在蓝框处搜索红框的组件添加
>
> ![image-20230301212358899](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230301212358899.png)
>
> 3.勾选右键新建显示的项目，window-perspective-customize Perspective As...
>
> ![image-20230301212755903](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230301212755903.png)
>
> 

## 六、面向对象OOP

> 三条主线
>
> - java类和类的成员：属性、方法、构造器、代码块、内部类
> - 面向对象的三大特征：封装、继承、多态
> - 其他关键字：this 、super、static、final、package、abstract等

> 什么是面向对象：强调具备了功能的对象，以类/对象为最小单位，考虑谁来做

### 6.1 类和对象

> 类是一类对事物的描述，是抽象的、概念上的定义
>
> 对象是实际存在的该类事物的每个个体，因此也称为实例
>
> 面向对象程序设计的重点就是类的设计
>
> 设计类，就是设计类的成员

> 类和对象的创建,以创建一个人的类为例
>
> - 如果创建了一个类的多个对象，则每个对象都独立的拥有一套类的属性(非static)
>
> ```java
> package com.hy.contact;
> 
> public class PersonTest {
>  public static void main(String[] args) {
>  	// 实例化一个人的对象
>  	Person person = new Person();
>  	// 调用对象的属性
>  	person.name = "Tom";
>  	person.age = 18;
>  	person.gender = true;
>  	System.out.println(person.name);
>  	System.out.println(person.age);
>  	System.out.println(person.gender);
>  	// 调用对象的方法
>  	person.eat();
>  	person.talk("Chinese");
>  	int a = person.walk();
>  	System.out.println(person.gender);
> 
>  }
> }
> 
> // 创建一个类，这个类代表一个人
> class Person{
> 	// 类有属性和方法两个基本成员
> 
> 	// 属性
> 	String name;
> 	int age;
> 	boolean gender;
> 
> 	// 方法，代表一个人的行为
> 	void eat() {
> 		System.out.println("人吃饭");
> 	}
> 	void talk(String language) {
> 		System.out.println("人说话，语言是：" + language);
> 	}
> 	int walk() {
> 		System.out.println("人行走");
> 		return 1;
> 	}	
> }
> ```
>
> 

> 对象的内存解析
>
> <img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230306201628194.png" alt="image-20230306201628194" style="zoom:67%;" />
>
> - 堆（Heap）： 所有的对象实例以及数组都要在堆上分配
> - 栈（stack）：是指虚拟机栈，用于存储局部变量。
> - 方法区（Method Area）：用于存储已经被虚拟机加载的类信息、常量、静态变量、即时编译器编译后的代码等数据。

### 6.2 属性和方法

#### 6.2.1 属性

> 从属性和局部变量的区别
>
>  *  不同点
>
>    1.在类中定义的位置不同
>
>    >  *  属性定义在类的大括号中
>    >  *  局部变量定义在方法、形参内
>
>    2.权限修饰符的不同
>
>    >  *  属性可以使用权限修饰符，如public，private ，缺省（不写），protected
>    >  *  局部变量不可以使用权限修饰符
>
>    3.初始值的不同
>
>    >  *  属性在声明时如果没有赋值，那么会自动赋初始值  String-null,整型-1,布尔-false,引用数据类型（类、数组、接口）-null,字符型char-0或'\u0000';
>    >  *  局部变量如不赋值会报错
>
>    4.在内存中加载的位置
>
>    >  *  属性加载到堆空间中
>    >  *  局部变量加载到栈空间中，随方法的结束自动销毁
>
>    ```java
>    // 代码示例
>    package com.hy.contact;
>    public class UserTest {
>        public static void main(String[] args) {
>        	User user = new User();
>        	System.out.println(user.name);     // name属性未赋初始值，则使用null
>        	
>        }
>    }
>    
>    class User{
>    	// 属性
>    	String name;
>    	int age;
>    	boolean gender;
>    	
>    	public void talk(String language) {   // language是形参，是局部变量
>    		System.out.println("我们使用" + language + "交流");
>    		
>    	}
>    	public void eat() {
>    		String food = "面食";     // 局部变量
>    		// public String food = "面食";   // 错误的
>    		System.out.println("北方人喜欢吃" + food);
>    	}
>    }
>    ```
>
>    

#### 6.2.2 方法

 * 定义一个方法一般需要以下部分

   权限修饰符  返回值类型  方法名（形参）{

   ​    方法体

   ​    return；

   }

 * 权限修饰符：public private 缺省 protected,修饰这个方法的调用权限

 * 返回值类型：方法有返回值，在这里写返回值的类型，没有返回值则写void，该部分必须写

 * 方法名：方法标识符，一般为驼峰式命名，需要做到见名知意。

 * 形参：调用该方法时的传入值，可以为空

 * 方法体：方法的具体功能

 * return：有两个功能，一个是返回值，另一个是结束方法

 * 根据有无返回值和形参将方法分为四类

   1、无返回值无形参

   2、有返回值无形参

   3、无返回值有形参

   4、有返回值有形参

   ```java
   package com.hy.contact;
   
   public class CustomTest {
   
   }
   
   class Custom{
   	// 四类方法
   	public void eat() {
   		System.out.println("吃饭");
   	}
   	public int walk() {
   		int footStep = 100;
   		System.out.println("散步");
   		return footStep;
   	}
   	public void sleep(int time) {
   		System.out.println("每天睡" + time);
   	}
   	public int addMethod(int a,int b) {
   		return a + b;
   	}
   	
   }
   ```

   

### 6.3 UML类图

> 在java开发中，可以使用UML类图来表达一个具体类
>
> 具体类在类图中用矩形框表示，矩形框分为三层：第一层是类名字。第二层是类的成员变量；第三层是类的方法。成员变量以及方法前的访问修饰符用符号来表示：
>
> - “+”表示 `public`；
> - “-”表示 `private`；
> - “#”表示 `protected`；
> - 不带符号表示 `default`。
>
> ```java
> class Custom{
> 	public String firstName;
> 	public String lastName;
>     private boolean c;
> 	// 四类方法
> 	public void eat() {
> 		System.out.println("吃饭");
> 	}
> 	public int walk() {
> 		int footStep = 100;
> 		System.out.println("散步");
> 		return footStep;
> 	}
> 	public void sleep(int time) {
> 		System.out.println("每天睡" + time);
> 	}
> 	public int addMethod(int a,int b) {
> 		return a + b;
> 	}	
> }
> ```
>
> 上面代码对应的类图
>
> <img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230306223632070.png" alt="image-20230306223632070" style="zoom:50%;" />
>
> 
>
> 
>
> 

