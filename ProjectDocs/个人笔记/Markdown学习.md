# Markdown学习

> Markdown文件的编写可以使用Typora软件编写

## 一、标题

#一级标题

##二级标题

###三级标题

以此类推



## 二、字体

**粗体:内容由两个星号括起来**     

*斜体*：内容由一个星号括起来

***斜体加粗：内容由 三个星号括起来***

~~删除：由两个波浪线括起来~~



## 三、引用

> 这句话是引用，由一个 > 起头



## 四、图片

> **由 \![]() 组成，[]中写图片名称，()写图片地址**



![截图](https://img2023.cnblogs.com/blog/1748933/202301/1748933-20230130205038745-311839382.png)

## 五、超链接
> **由 \[超链接标题](超链接地址) 组成，不要丢掉https://**

[点此跳转到百度搜索](https://www.baidu.com)



## 六、列表

> **1. 表示有序列表**

1. A
2. B
3. C

> -表示无序列表

- A
- B
- C



## 七、表格

> 表格较为复杂，可以直接右键插入表格

| 姓名 | 年龄 | 职业     |
| ---- | ---- | -------- |
| 张三 | 43   | 法外狂徒 |

> **具体写法为，在源代码中删除空白行**

| 姓名 | 年龄 | 职业| 

| -- | -- | -- | 

| 张三 | 43 | 法外狂徒 |



## 八、代码

> 三个点，在Tab上面

``` java
public class helloWorld{
    public static void main(String[] args){
        System.out.println("Hello World");
    }

```

