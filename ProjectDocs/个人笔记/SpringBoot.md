# Spring Boot

MVC架构

> Model：数据
>
> Controller：控制器，负责接受和处理HTTP请求
>
> View：页面

![image-20230403145901819](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403145901819.png)



## 一、快速创建

① 创建Maven项目

使用IDEA，新建maven项目

② 导入SpringBoot起步依赖

修改pom.xml，

因为依赖每个版本都不一样，所以需要每次都去官网上复制

https://docs.spring.io/spring-boot/docs/2.6.11/maven-plugin/reference/htmlsingle/#?

```
<parent>
    <groupId>org.springframework.boot</groupId>
    <artifactId>spring-boot-starter-parent</artifactId>
    <version>2.6.11</version>
</parent>

<dependencies>
    <dependency>
        <!-- Import dependency management from Spring Boot -->
        <groupId>org.springframework.boot</groupId>
        <artifactId>spring-boot-starter-web</artifactId>
    </dependency>
</dependencies>
```

③ 定义Controller
④ 编写引导类

引导类包含main

```java
package com.itheima;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;

@SpringBootApplication
public class HelloApplication {
    public static void main(String[] args) {
        SpringApplication.run(HelloApplication.class,args);
    }
}
```

⑤ 启动测试  



配置热部署

> - 在实际的项目开发调试过程中会频繁地修改后台类文件，导致需要重新编译、
>   重新启动，整个过程非常麻烦，影响开发效率。
> - Spring Boot提供了spring-boot-devtools组件，使得无须手动重启Spring
>   Boot应用即可重新编译、启动项目，大大缩短编译启动的时间。
> -  devtools会监听classpath下的文件变动，触发Restart类加载器重新加载该类，
>   从而实现类文件和属性文件的热部署。
> -  并不是所有的更改都需要重启应用（如静态资源、视图模板），可以通过设置
>   spring.devtools.restart.exclude属性来指定一些文件或目录的修改不用重启应
>   用  

添加依赖

```xml
<dependency>
    <groupId>org.springframework.boot</groupId>
    <artifactId>spring-boot-devtools</artifactId>
    <optional>true</optional>
</dependency>
```

在配置文件中配置devtools

```xml
spring:
  devtools:
    restart:
      # 热部署生效
      enabled: true
      # 设置重启目录
      additional-paths: src/main/java
      # 设置class path目录下的web-inf文件夹内容修改不重启
      exclude: static/**
```

打开Settings页面，在左边的菜单栏依次找到Build,Execution,Deployment→Compile，勾选Build project automatically

按Ctrl+Shift+Alt+/快捷键调出Maintenance页面，单击Registry，勾选
compiler.automake.allow.when.app.running复选框。  注意2021之后的版本在settings->advanced settings勾选Allow auto-make to start even if developed application is currnetly runing

## 二、控制器

> Spring Boot提供了@Controller和@RestController两种注解来标识此类负责
> 接收和处理HTTP请求。
> 如果请求的是页面和数据，使用@Controller注解即可；如果只是请求数据，
> 则可以使用@RestController注解。  



路由映射

> @RequestMapping注解主要负责URL的路由映射。它可以添加在Controller
> 类或者具体的方法上。
> 如果添加在Controller类上，则这个Controller中的所有路由映射都将会加上此
> 映射规则，如果添加在方法上，则只对当前方法生效。
> @RequestMapping注解包含很多属性参数来定义HTTP的请求映射规则。常
> 用的属性参数如下：
>
> - value: 请求URL的路径，支持URL模板、正则表达式
>
> - method: HTTP请求方法
>
> - consumes: 请求的媒体类型（Content-Type），如application/json
>
> - produces: 响应的媒体类型
>
> - params，headers: 请求的参数及请求头的值  

```java
    @RequestMapping(value = "/getTest1",method = RequestMethod.GET)
    public String getTest1(){
        return "GET请求";
    }

    @RequestMapping(value = "/getTest2",method = RequestMethod.GET)
//  http://localhost:8080/getTest2?nickname=xxx&phone=xxx
    public String getTest2(String nickname,String phone){
        System.out.println("nickname:"+nickname);
        System.out.println("phone:"+phone);
        return "GET请求";
    }
// 获取路径中的参数
    @GetMapping("/user/{id}")
    public String getUserById(@PathVariable int id){
        System.out.println(id);
        return "根据ID获取用户信息";
    }

    @RequestMapping(value = "/getTest3",method = RequestMethod.GET)
//  http://localhost:8080/getTest2?nickname=xxx
// 添加RequestParam,则默认该参数为非空
    public String getTest3(@RequestParam(value = "nickname",required = false) String name){
        System.out.println("nickname:"+name);
        return "GET请求";
    }

    @RequestMapping(value = "/postTest1",method = RequestMethod.POST)
    public String postTest1(){
        return "POST请求";
    }
    @RequestMapping(value = "/postTest2",method = RequestMethod.POST)
    public String postTest2(String username,String password){
        System.out.println("username:"+username);
        System.out.println("password:"+password);
        return "POST请求";
    }
    @RequestMapping(value = "/postTest3",method = RequestMethod.POST)
    public String postTest3(User user){
        System.out.println(user);
        return "POST请求";
    }

    // 前端传来的是json格式的数据
    @RequestMapping(value = "/postTest4",method = RequestMethod.POST)
    public String postTest4(@RequestBody User user){
        System.out.println(user);
        return "POST请求";
    }

    @GetMapping("/test/*")
    public String test(){
        return "通配符请求";
    }
```



## 三、配置文件

- SpringBoot提供了2种配置文件类型： properteis和yml/yaml
- 默认配置文件名称： application
- 在同一级目录下优先级为： properties > yml > yaml  

YAML：基本语法

- 大小写敏感
-  数据值前边必须有空格，作为分隔符
-  使用缩进表示层级关系
-  缩进时不允许使用Tab键，只允许使用空格（各个系统 Tab对应的 空格数目可能不同，导致层次混乱） 。
-  缩进的空格数目不重要，只要相同层级的元素左侧对齐即可
- “#” 表示注释，从这个字符一直到行尾，都会被解析器忽略。  

YAML：数据格式

- 对象(map)：键值对的集合。
-  数组：一组按次序排列的值
-  纯量：单个的、不可再分的值
- 参数引用

```yaml
person:
name: zhangsan
\# 行内写法
person: {name: zhangsan}
address:
\- beijing
\- shanghai
\# 行内写法
address: [beijing,shanghai]
msg1: 'hello \n world'   # 单引忽略转义字符
msg2: "hello \n world"   # 双引识别转义字符
name: lisi
person:
    name: ${name} # 引用上边定义的name值
```

读取配置内容
1） @Value

```java
    @Value("${name}")
    private String name;
    @Value("${person.name}")
    private String p_name;
    @Value("${address[0]}")
    private String address0;
```

2） Environment

```java
@Autowired
private Environment environment;

System.out.println(environment.getProperty("name"));
```

3） @ConfigurationProperties  

> 多用于配置文件中对象的提取,首先创建一个对象类，使用 @ConfigurationProperties  这个对象会自动到配置文件中取值，之后可以直接定义类使用对象的值

```java

package com.example.demo;

import lombok.Data;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.stereotype.Component;

@Component
@ConfigurationProperties(prefix = "person") // prefix,将参数对应的对象注入，如果没有就只注入纯量
@Data
public class Person {
    private String name;
}
```



## profile

我们在开发Spring Boot应用时，通常同一套程序会被安装到不同环境，比如：开发、测试、生产等。其中数据库地址、服务
器端口等等配置都不同，如果每次打包时，都要修改配置文件，那么非常麻烦。 profile功能就是来进行动态配置切换的。
1） profile配置方式

- 多profile文件方式

> 创建多个properties文件，分别命名为application-dev，application-pro，application-test，

- yml多文档方式

> 创建一个application.yml文件，每部分用---隔开

![image-20230329151622359](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230329151622359.png)

2）profile激活方式

-  配置文件

> properties和yml的激活方式相同，spring.profiles.active:要激活的profile

-  虚拟机参数

> 在VM options 指定： -Dspring.profiles.active=dev  

![image-20230329151359342](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230329151359342.png)

-  命令行参数  

> java –jar xxx.jar --spring.profiles.active=dev  

![image-20230329151340920](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230329151340920.png)

> 也可以在在运行jar包时指定profile

将项目打包

![image-20230329152055241](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230329152055241.png)

运行项目

![image-20230329151527233](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230329151527233.png)

### 加载顺序

#### 内部配置加载顺序

Springboot程序启动时，会从以下位置加载配置文件：

> file:当前项目目录，classpath：resource目录

\1. file:./config/：当前项目下的/config目录下
\2. file:./ ：当前项目的根目录
\3. classpath:/config/： classpath的/config目录
\4. classpath:/ ： classpath的根目录
加载顺序为上文的排列顺序，高优先级配置的属性会生效  

> 注意，直接使用spring Initializr创建的项目没有file目录，需要先创建一个空项目，spring作为空项目的一个模块，此时创建的spring模块没有maven需要在pom.xml文件上右键选择add maven

![image-20230329164445258](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230329164445258.png)



外部配置加载顺序
通过官网查看外部属性加载顺序：
https://docs.spring.io/spring-boot/docs/current/reference/html/boot-features-external-config.html  

> ##  2. Externalized Configuration
>
> Spring Boot lets you externalize your configuration so that you can work with the same application code in different environments. You can use a variety of external configuration sources including Java properties files, YAML files, environment variables, and command-line arguments.
>
> Property values can be injected directly into your beans by using the `@Value` annotation, accessed through Spring’s `Environment` abstraction, or be [bound to structured objects](https://docs.spring.io/spring-boot/docs/current/reference/html/features.html#features.external-config.typesafe-configuration-properties) through `@ConfigurationProperties`.
>
> Spring Boot uses a very particular `PropertySource` order that is designed to allow sensible overriding of values. Later property sources can override the values defined in earlier ones. Sources are considered in the following order:
>
> 1. Default properties (specified by setting `SpringApplication.setDefaultProperties`).
> 2. [`@PropertySource`](https://docs.spring.io/spring-framework/docs/6.0.7/javadoc-api/org/springframework/context/annotation/PropertySource.html) annotations on your `@Configuration` classes. Please note that such property sources are not added to the `Environment` until the application context is being refreshed. This is too late to configure certain properties such as `logging.*` and `spring.main.*` which are read before refresh begins.
> 3. Config data (such as `application.properties` files).
> 4. A `RandomValuePropertySource` that has properties only in `random.*`.
> 5. OS environment variables.
> 6. Java System properties (`System.getProperties()`).
> 7. JNDI attributes from `java:comp/env`.
> 8. `ServletContext` init parameters.
> 9. `ServletConfig` init parameters.
> 10. Properties from `SPRING_APPLICATION_JSON` (inline JSON embedded in an environment variable or system property).
> 11. Command line arguments.
> 12. `properties` attribute on your tests. Available on [`@SpringBootTest`](https://docs.spring.io/spring-boot/docs/3.0.5/api/org/springframework/boot/test/context/SpringBootTest.html) and the [test annotations for testing a particular slice of your application](https://docs.spring.io/spring-boot/docs/current/reference/html/features.html#features.testing.spring-boot-applications.autoconfigured-tests).
> 13. [`@TestPropertySource`](https://docs.spring.io/spring-framework/docs/6.0.7/javadoc-api/org/springframework/test/context/TestPropertySource.html) annotations on your tests.
> 14. [Devtools global settings properties](https://docs.spring.io/spring-boot/docs/current/reference/html/using.html#using.devtools.globalsettings) in the `$HOME/.config/spring-boot` directory when devtools is active.



> 常用的外部配置为命令行配置，即在命令行运行jar包是添加配置参数
>
> --server.port=8083
>
> 参数过多时，可以指定配置文件，配置文件可放置磁盘任意位置
>
> --spring.config.location=e://application.properties
>
> 当配置文件与jar包同级时，无需参数将自动识别

## 四、整合其他框架

### SpringBoot整合Junit。

```xml
//① 搭建SpringBoot工程
//② 引入starter-test起步依赖
		<dependency>
			<groupId>org.springframework.boot</groupId>
			<artifactId>spring-boot-starter-test</artifactId>
			<scope>test</scope>
		</dependency>
		<dependency>
			<groupId>junit</groupId>
			<artifactId>junit</artifactId>
			<scope>test</scope>
		</dependency>
```



  ```java
  
  //③ 编写测试类
  //④ 添加测试相关注解
  //• @RunWith(SpringRunner.class)
  //• @SpringBootTest(classes = 启动类.class)
  //⑤ 编写测试方法
  
  package com.why.test;
  
  import com.why.configurationtest.ConfigurationTestApplication;
  import com.why.configurationtest.UserService;
  import org.junit.Test;
  import org.junit.runner.RunWith;
  import org.springframework.beans.factory.annotation.Autowired;
  import org.springframework.boot.test.context.SpringBootTest;
  import org.springframework.test.context.junit4.SpringRunner;
  
  @RunWith(SpringRunner.class)
  @SpringBootTest(classes=ConfigurationTestApplication.class)   //ConfigurationTestApplication.class:在主函数中
  public class UserServiceTest {
      @Autowired
      private UserService userService;
      @Test
      public void testAdd(){
          userService.add();
      }
  }
  
  
  ```

### SpringBoot整合redis

> 安装redis，见Linux学习
>
> ① 搭建SpringBoot工程
> ② 引入redis起步依赖
>
> ```xml
> <dependency>
>    <groupId>org.springframework.data</groupId>
>    <artifactId>spring-data-redis</artifactId>
>    <version>2.5.10</version>
>    <scope>test</scope>
> </dependency>
> ```
>
> ③ 配置redis相关属性
>
> ```xml
> spring:
>   redis:
>     host: 192.168.8.129
>     port: 6379
> ```
>
> ④ 注入RedisTemplate模板，需要安装，点击提示即可
>
> ```
> @Resource
>     private RedisTemplate redisTemplate;⑤ 编写测试方法，测试  
> ```
>
> ⑤ 编写测试方法，测试  
>
> ```java
> package com.why.test;
> 
> import com.why.configurationtest.ConfigurationTestApplication;
> import org.junit.Test;
> import org.junit.runner.RunWith;
> import org.springframework.beans.factory.annotation.Autowired;
> import org.springframework.boot.test.context.SpringBootTest;
> import org.springframework.data.redis.core.RedisTemplate;
> import org.springframework.test.context.junit4.SpringRunner;
> 
> import javax.annotation.Resource;
> 
> @RunWith(SpringRunner.class)
> @SpringBootTest(classes= ConfigurationTestApplication.class)
> public class redisTest {
>     @Resource
>     private RedisTemplate redisTemplate;
>     @Test
>     public void testSet(){
>         redisTemplate.boundValueOps("name").set("zhangsan");
>     }
>     @Test
>     public void testGet(){
>         Object name = redisTemplate.boundValueOps("name").get();
>         System.out.println(name);
>     }
> }
> ```

### 整合MyBatis

1. 添加依赖

```xml
<?xml version="1.0" encoding="UTF-8"?>
<project xmlns="http://maven.apache.org/POM/4.0.0" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:schemaLocation="http://maven.apache.org/POM/4.0.0 https://maven.apache.org/xsd/maven-4.0.0.xsd">
    <modelVersion>4.0.0</modelVersion>
    <parent>
        <groupId>org.springframework.boot</groupId>
        <artifactId>spring-boot-starter-parent</artifactId>
        <version>2.6.11</version>
    </parent>
    <groupId>com.example</groupId>
    <artifactId>demo14</artifactId>
    <version>0.0.1-SNAPSHOT</version>
    <name>demo14</name>
    <description>demo14</description>
    <properties>
        <java.version>8</java.version>
    </properties>
    <dependencies>
        <dependency>
            <groupId>org.springframework.boot</groupId>
            <artifactId>spring-boot-starter-web</artifactId>
        </dependency>

        <dependency>
            <groupId>org.mybatis.spring.boot</groupId>
            <artifactId>mybatis-spring-boot-starter</artifactId>
            <version>2.1.3</version>
        </dependency>

        <dependency>
            <groupId>mysql</groupId>
            <artifactId>mysql-connector-java</artifactId>
        </dependency>

        <dependency>
            <groupId>org.springframework.boot</groupId>
            <artifactId>spring-boot-starter-test</artifactId>
            <scope>test</scope>
        </dependency>

        <dependency>
            <groupId>org.projectlombok</groupId>
            <artifactId>lombok</artifactId>
        </dependency>
    </dependencies>

    <build>
        <plugins>
            <plugin>
                <groupId>org.springframework.boot</groupId>
                <artifactId>spring-boot-maven-plugin</artifactId>
            </plugin>
        </plugins>
    </build>

</project>

```

2. 配置yml

```
spring:
  datasource:
    driver-class-name: com.mysql.jdbc.Driver
    url: JDBC:mysql://192.168.8.132:3306/springboot?serverTimezone=GMT%2B8&useSSL=true
    username: root
    password: 123456

```

3. 创建domain包，并添加实例类

<img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230330212217504.png" alt="image-20230330212217504" style="zoom:50%;" />

```java
package com.example.demo14.domain;

import lombok.Data;
import org.springframework.stereotype.Repository;

@Data
@Repository
public class User {
    private String user;
    private String passWord;
}
```

4. 创建Mapper包，并添加Mapper文件（注意文件类型为Interface）

<img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230330212333275.png" alt="image-20230330212333275" style="zoom:50%;" />

```java
package com.example.demo14.Mapper;

import com.example.demo14.domain.User;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Select;

import java.util.List;

@Mapper
public interface UserMapper {
    @Select("select * from t_user")
    public List<User> findAll();
}
```

5. 在测试类中编写测试函数

```java
package com.example.demo14;

import com.example.demo14.Mapper.UserMapper;
import com.example.demo14.domain.User;
import org.junit.jupiter.api.Test;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.test.context.SpringBootTest;

import java.util.List;

@SpringBootTest
class Demo14ApplicationTests {
    @Autowired
    private UserMapper userMapper;
    @Test
    void contextLoads() {
        List<User> list = userMapper.findAll();
        System.out.println(list);
    }

}
```



> 前一种方法是在Mapper文件中写SQL语句，下面使用XML文件进行映射

1. 建立一个新的Mapper文件，与前一个的区别只有方法不用添加任何注解

```java
package com.example.demo14.Mapper;

import com.example.demo14.domain.User;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Select;

import java.util.List;

@Mapper
public interface UserXmlMapper {
    public List<User> findAll();
}

```

2. 在resources文件夹下创建Mapper文件夹，在Mapper文件夹下创建xml文件，文件名格式为*Mapper.xml，方便在yml文件中注册

![image-20230330215046908](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230330215046908.png)



```xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE mapper PUBLIC "-//mybatis.org//DTD Mapper 3.0//EN" "http://mybatis.org/dtd/mybatis-3-mapper.dtd">
<mapper namespace="com.example.demo14.Mapper.UserXmlMapper">
    <select id="findAll" resultType="user">
        select * from t_user
    </select>
</mapper>
```

> namespce的路径为被映射的Mapper文件的
>
> <img src="C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230330215208724.png" alt="image-20230330215208724" style="zoom:50%;" />

3. 在yml中注册xml

```
mybatis:
  mapper-locations: classpath:mapper/*.xml
  type-aliases-package: com.example.demo14.domain
```

> type-aliases-package的路径为实体类的Reference

### 整合MyBatis Plus

- 添加依赖

```
<!--    mybatis-plus依赖    -->
<dependency>
    <groupId>com.baomidou</groupId>
    <artifactId>mybatis-plus-boot-starter</artifactId>
    <version>3.4.2</version>
</dependency>
<!--    数据连接池 druid    -->
<dependency>
    <groupId>com.alibaba</groupId>
    <artifactId>druid-spring-boot-starter</artifactId>
    <version>1.1.20</version>
</dependency>
<!-- 添加mysql -->
<dependency>
    <groupId>mysql</groupId>
    <artifactId>mysql-connector-java</artifactId>
</dependency>
```

- 配置同mybatis

```
  datasource:
    driver-class-name: com.mysql.jdbc.Driver
    url: JDBC:mysql://192.168.8.132:3306/springboot?serverTimezone=GMT%2B8&useSSL=true
    username: root
    password: 123456
    type: com.alibaba.druid.pool.DruidDataSource
```

- 在启动类添加@MapperScan("Mapper文件夹的路径")
- 编写实体类

```java
@TableName("t_user")
@Data
public class User {
    private String username;
    private String password;
}
```

- 编写Mapper包下的UserMapper

```java
package com.example.demo.mapper;

import com.baomidou.mybatisplus.core.mapper.BaseMapper;
import com.example.demo.domain.User;
import org.springframework.stereotype.Repository;
// 注意这里的User既是表名也是类名,如果表名和类名不一致，需要在类名上添加注解@TableName("t_user")
@Repository
public interface UserMapper extends BaseMapper<User> {
}
```

- 使用

```java
@Autowired
private UserMapper userMapper;
@GetMapping("/query")
public List<User> query(){
    List<User> list = userMapper.selectList(null);
    return list;
}
```

### 多表查询

实现复杂关系映射，可以使用@Results注解，@Result注解，@One注解，@Many注解组合完成复杂关系的配置。

![image-20230403222415666](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403222415666.png)  

> 注意这里使用的多表查询是MyBatis技术，Mybatis-plus只是对单表操作进行了增强

> 举例，需要查询一个用户的对应的多个订单信息

因为使用的是MyBatis技术，所以需要自己写映射

首先在user实体类中添加order对象，并注明该字段不存在与user表中

```java
package com.example.demo.domain;

import com.baomidou.mybatisplus.annotation.IdType;
import com.baomidou.mybatisplus.annotation.TableField;
import com.baomidou.mybatisplus.annotation.TableId;
import com.baomidou.mybatisplus.annotation.TableName;
import lombok.Data;

import java.util.List;

@TableName("t_user")
@Data
public class User {
    @TableId(type= IdType.AUTO)
    private Integer id;
    private String username;
    private String password;
    @TableField(exist = false)   //指明这个字段在这个表中是不存在的
    private List<Order> orders;
}
```

创建order实体类

```java
package com.example.demo.domain;

import com.baomidou.mybatisplus.annotation.TableField;
import com.baomidou.mybatisplus.annotation.TableName;
import lombok.Data;
@TableName("t_order")
@Data
public class Order {
    private Integer id;
    private Integer amount;
    private String merchandise;
    private Integer uid;
    @TableField(exist = false)
    private User user;

}
```

修改user的mapper文件

```java
package com.example.demo.mapper;

import com.baomidou.mybatisplus.core.mapper.BaseMapper;
import com.example.demo.domain.User;
import org.apache.ibatis.annotations.Many;
import org.apache.ibatis.annotations.Result;
import org.apache.ibatis.annotations.Results;
import org.apache.ibatis.annotations.Select;
import org.springframework.stereotype.Repository;

import java.util.List;

@Repository
public interface UserMapper extends BaseMapper<User> {
    //   查询用户，根据用户id查询信息   select * from user where id =
    @Select("select * from t_user where id = #{id}")
    User selectById(int id);


    //   查询用户及其所有的订单
    @Select("select * from t_user")
    @Results(
            {
                    @Result(column = "id",property = "id"),
                    @Result(column = "username",property = "username"),
                    @Result(column = "password",property = "password"),
                //调用了order的mapper文件中的selectByUid方法
                    @Result(column = "id",property = "orders",javaType = List.class,
                            many=@Many(select = "com.example.demo.mapper.OrderMapper.selectByUid")
                    )
            }
    )
    List<User> selectAllUserAndOrders();
}
```

修改order的mapper文件

```java
package com.example.demo.mapper;

import com.baomidou.mybatisplus.core.mapper.BaseMapper;
import com.example.demo.domain.Order;
import com.example.demo.domain.User;
import org.apache.ibatis.annotations.One;
import org.apache.ibatis.annotations.Result;
import org.apache.ibatis.annotations.Results;
import org.apache.ibatis.annotations.Select;

import java.util.List;

public interface OrderMapper extends BaseMapper<Order> {

    @Select("select * from t_order where uid = #{uid}")
    List<Order> selectByUid(int uid);

    //  查询所有的订单，同时查询订单的用户
    @Select("select * from t_order")
    @Results({
            @Result(column = "id",property = "id"),
            @Result(column = "amount",property = "amount"),
            @Result(column = "merchandise",property = "merchandise"),
            @Result(column = "uid",property = "user",javaType = User.class,
                    one=@One(select = "com.example.demo.mapper.UserMapper.selectById")
            )
    })
    List<Order> selectAllOrdersAndUser();
}
```

在controller中使用

```java
@GetMapping("/query")
public List<User> query(){
    List<User> list = userMapper.selectAllUserAndOrders();
    return list;
}
```

### 条件查询

```java
//  条件查询
    @GetMapping("/user/find")
    public List<User> findByCond(){
        QueryWrapper<User> queryWrapper = new QueryWrapper();
        queryWrapper.eq("username","zhangsan");
        return userMapper.selectList(queryWrapper);
    }
```

### 分页查询

编写配置文件

```java
@Configuration
public class MyBatisPlusConfig {
    @Bean
    public MybatisPlusInterceptor paginationInterceptor() {
        MybatisPlusInterceptor interceptor = new MybatisPlusInterceptor();
        // 数据库
        PaginationInnerInterceptor paginationInterceptor = new PaginationInnerInterceptor(DbType.MYSQL);
        interceptor.addInnerInterceptor(paginationInterceptor);
        return interceptor;

    }
}
```

使用分页查询

```java
@GetMapping("/user/findAll")
public IPage getAllUser(){
    Page<User> page = new Page<>(0,2);
    IPage iPage = userMapper.selectPage(page,null);
    return iPage;
}
```

## 静态资源访问

> 使用IDEA创建Spring Boot项目，会默认创建出classpath:/static/目录，静态
> 资源一般放在这个目录下即可。在浏览器访问静态资源localhost:8080/资源名称
>
> 如果默认的静态资源过滤策略不能满足开发需求，也可以自定义静态资源过滤
> 策略。
>
> ![image-20230403160914303](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403160914303.png)application.properties中直接定义过滤规则和静态资源位置：
> 过滤规则为/static/**，静态资源位置为classpath:/static/  

## 文件上传

Spring Boot工程嵌入的tomcat限制了请求的文件大小，每个文件的配置最大为1Mb，单次请求的文件的总数不能大于10Mb。
要更改这个默认值需要在配置文件（如application.properties）中加入两个配置  

![image-20230403161510454](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403161510454.png)

```
spring:
  devtools:
    restart:
      # 热部署生效
      enabled: true
      # 设置重启目录
      additional-paths: src/main/java
      # 设置class path目录下的web-inf文件夹内容修改不重启
      exclude: static/**
  mvc:
    # 配置该项时上传到upload的文件也使用/static/文件名 来访问
    static-path-pattern: /static/**
  web:
    resources:
      static-locations: classpath:/static/,/upload/
  servlet:
    multipart:
      max-file-size: 10MB
      max-request-size: 10MB
```

```java
package com.example.demo.controller;

import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.multipart.MultipartFile;

import javax.servlet.http.HttpServletRequest;
import java.io.File;
import java.io.IOException;

@RestController
public class FileController {
    private static final String UPLOAD_FOLDER = System.getProperty("user.dir") + "/upload/";
    @PostMapping("/up")
    public String upload(String nickname, MultipartFile f, HttpServletRequest request) throws IOException{
        System.out.println("文件大小:" + f.getSize());
        System.out.println("文件类型:" + f.getContentType());
        System.out.println("文件名:" + f.getOriginalFilename());
        System.out.println(System.getProperty("user.dir"));

        String path = request.getServletContext().getRealPath("/upload/");   //获取服务器动态路径
        System.out.println(path);
        saveFile(f,path);
        return "文件上传成功";
    }
    public void saveFile(MultipartFile f,String path) throws IOException{
        File upDir = new File(path);
        if(!upDir.exists()){
            upDir.mkdir();
        }
        File file = new File(path + f.getOriginalFilename());
        f.transferTo(file);
    }
}

```

## 拦截器

拦截器在Web系统中非常常见，对于某些全局统一的操作，我们可以把它提取到拦截器中实现。总结起来，拦截器大致有以下几种使用场景：

- 权限检查：如登录检测，进入处理程序检测是否登录，如果没有，则直接返回登录页面。

- 性能监控：有时系统在某段时间莫名其妙很慢，可以通过拦截器在进入处理程序之前记录开始时间，在处理完后记录结束时间，从而得到该请求的处理时间

- 通用行为：读取cookie得到用户信息并将用户对象放入请求，从而方便后续流程使用，还有提取Locale、Theme信息等，只要是多个处理程序都需要的，即可使用拦截器实现。  



> Spring Boot定义了HandlerInterceptor接口来实现自定义拦截器的功能HandlerInterceptor接口定义了preHandle、postHandle、afterCompletion
> 三种方法，通过重写这三种方法实现请求前、请求后等操作  

![image-20230403180812423](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403180812423.png)

- 定义拦截器

> 拦截器的名称通常以Interceptor结尾

```java
package com.example.demo.Interceptor;

import org.springframework.web.servlet.HandlerInterceptor;

import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

public class LoginInterceptor implements HandlerInterceptor {
    @Override
    public boolean preHandle(HttpServletRequest request, HttpServletResponse response, Object handler) throws Exception {
        System.out.println("LoginInterceptor");
        return true;
    }
}

```

- 注册拦截器

- addPathPatterns方法定义拦截的地址

- excludePathPatterns定义排除某些地址不被拦截

- 添加的一个拦截器没有addPathPattern任何一个url则默认拦截所有请求

- 如果没有excludePathPatterns任何一个请求，则默认不放过任何一个请求。  

```java
package com.example.demo.config;

import com.example.demo.Interceptor.LoginInterceptor;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.InterceptorRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;

@Configuration
public class WebConfig implements WebMvcConfigurer {
    @Override
    public void addInterceptors(InterceptorRegistry registry) {
        registry.addInterceptor( new LoginInterceptor()).addPathPatterns("/user/**");  // 添加要拦截的请求
    }
}
```

## RESTful服务

- RESTful是目前流行的互联网软件服务架构设计风格。

- REST（Representational State Transfer，表述性状态转移）一词是由RoyThomas Fielding在2000年的博士论文中提出的，它定义了互联网软件服务的
  架构原则，如果一个架构符合REST原则，则称之为RESTful架构。

- REST并不是一个标准，它更像一组客户端和服务端交互时的架构理念和设计原则，基于这种架构理念和设计原则的Web API更加简洁，更有层次。  

### 特点

- 每一个URI代表一种资源

- 客户端使用GET、POST、PUT、DELETE四种表示操作方式的动词对服务端资源进行操作：GET用于获取资源，POST用于新建资源（也可以用于更新资源），PUT用于更新资源，DELETE用于删除资源。

- 通过操作资源的表现形式来实现服务端请求操作。

- 资源的表现形式是JSON或者HTML。

- 客户端与服务端之间的交互在请求之间是无状态的，从客户端到服务端的每个请求都包含必需的信息。  

### RESTful API

符合RESTful规范的Web API需要具备如下两个关键特性：

- 安全性：安全的方法被期望不会产生任何副作用，当我们使用GET操作获取资源时，不会引起资源本身的改变，也不会引起服务器状态的改变。

- 幂等性：幂等的方法保证了重复进行一个请求和一次请求的效果相同（并不是指响应总是相同的，而是指服务器上资源的状态从第一次请求后就不再改变了），在数学上幂等性是指N次变换和一次变换相同。  

### Http Method

- HTTP提供了POST、GET、PUT、DELETE等操作类型对某个Web资源进行Create、Read、Update和Delete操作。

- 一个HTTP请求除了利用URI标志目标资源之外，还需要通过HTTP Method指定针对该资源的操作类型，一些常见的HTTP方法及其在RESTful风格下的使用：  

![image-20230403183838181](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403183838181.png)

### Http状态码

HTTP状态码就是服务向用户返回的状态码和提示信息，客户端的每一次请求，服务都必须给出回应，回应包括HTTP状态码和数据两部分。
HTTP定义了40个标准状态码，可用于传达客户端请求的结果。状态码分为以下5个类别：

- 1xx：信息，通信传输协议级信息

- 2xx：成功，表示客户端的请求已成功接受

- 3xx：重定向，表示客户端必须执行一些其他操作才能完成其请求

- 4xx：客户端错误，此类错误状态码指向客户端

- 5xx：服务器错误，服务器负责这写错误状态码  

RESTful API中使用HTTP状态码来表示请求执行结果的状态，适用于REST API设计的代码以及对应的HTTP方法。  

![image-20230403183938164](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403183938164.png)

### spring boot实现RESTful API

Spring Boot提供的spring-boot-starter-web组件完全支持开发RESTful API，提供了与REST操作方式（GET、POST、PUT、DELETE）对应的注解。
@GetMapping：处理GET请求，获取资源。
@PostMapping：处理POST请求，新增资源。
@PutMapping：处理PUT请求，更新资源。
@DeleteMapping：处理DELETE请求，删除资源。
@PatchMapping：处理PATCH请求，用于部分更新资源。  

在RESTful架构中，每个网址代表一种资源，所以URI中建议不要包含动词，只包含名词即可，而且所用的名词往往与数据库的表格名对应。用户管理模块API示例：  

![image-20230403184044252](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403184044252.png)



## Swagger

Swagger是一个规范和完整的框架，用于生成、描述、调用和可视化RESTful风格的Web服务，是非常流行的API表达工具。
Swagger能够自动生成完善的RESTful API文档，，同时并根据后台代码的修改同步更新，同时提供完整的测试页面来调试API。  

### 在spring boot中使用Swagger

- 添加依赖

```
<!-- 添加swagger2相关功能 -->
<dependency>
    <groupId>io.springfox</groupId>
    <artifactId>springfox-swagger2</artifactId>
    <version>2.9.2</version>
</dependency>
<!-- 添加swagger-ui相关功能 -->
<dependency>
    <groupId>io.springfox</groupId>
    <artifactId>springfox-swagger-ui</artifactId>
    <version>2.9.2</version>
</dependency>
```

Spring Boot 2.6.X后与Swagger有版本冲突问题，需要在
application.properties中加入以下配置：  

![image-20230403185506148](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403185506148.png)

- 配置Swagger

```java
package com.example.demo.config;

import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.servlet.config.annotation.ResourceHandlerRegistry;
import org.springframework.web.servlet.config.annotation.WebMvcConfigurer;
import springfox.documentation.builders.ApiInfoBuilder;
import springfox.documentation.builders.PathSelectors;
import springfox.documentation.builders.RequestHandlerSelectors;
import springfox.documentation.service.ApiInfo;
import springfox.documentation.spi.DocumentationType;
import springfox.documentation.spring.web.plugins.Docket;
import springfox.documentation.swagger2.annotations.EnableSwagger2;

@Configuration // 告诉Spring容器，这个类是一个配置类
@EnableSwagger2 // 启用Swagger2功能
public class SwaggerConfig implements WebMvcConfigurer {
    /**
     * 配置Swagger2相关的bean
     */
    @Bean
    public Docket createRestApi() {
        return new Docket(DocumentationType.SWAGGER_2)
                .apiInfo(apiInfo())
                .select()
                .apis(RequestHandlerSelectors.basePackage("com"))// com包下所有API都交给Swagger2管理
                .paths(PathSelectors.any()).build();
    }

    /**
     * 此处主要是API文档页面显示信息
     */
    private ApiInfo apiInfo() {
        return new ApiInfoBuilder()
                .title("演示项目API") // 标题
                .description("演示项目") // 描述
                .version("1.0") // 版本
                .build();
    }

    @Override
    public void addResourceHandlers(ResourceHandlerRegistry registry) {
        registry.addResourceHandler("/**").addResourceLocations("classpath:/static/");
        registry.addResourceHandler("swagger-ui.html").addResourceLocations("classpath:/META-INF/resources/");
        registry.addResourceHandler("doc.html").addResourceLocations("classpath:/META-INF/resources/");
        registry.addResourceHandler("/webjars/**").addResourceLocations("classpath:/META-INF/resources/webjars/");
    }
}


```

- 使用

启动项目访问 http://127.0.0.1:8080/swagger-ui.html ，即可打开自动生成的可视化测试页面  

### Swagger常用注解

![image-20230403192758859](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230403192758859.png)

## 问题

1、没有run

> 点击

![image-20230328112726449](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230328112726449.png)

2、java: 警告: 源发行版 17 需要目标发行版 17

> 改为对应版本

![image-20230328112638585](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230328112638585.png)

![image-20230328112702249](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230328112702249.png)

3、使用@Data前安装 LokBok插件

4、java:提示程序包org.junit不存在时的解决方法

如下图：

![img](https://img-blog.csdnimg.cn/5d71701269ae47279793d03f629c4660.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA6ZSZ5Zyo6LSq5oGL5L2g5rip5p-U,size_20,color_FFFFFF,t_70,g_se,x_16)

 3号这个地方勾选一下就好了，勾选完就能运行了，也不提示没有包了。
