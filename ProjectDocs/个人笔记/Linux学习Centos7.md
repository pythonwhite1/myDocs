# Linux学习（Centos7）

- 最小化安装

- 连接网络

> 最小化安装无法使用ifconfig
>
> 1. vi /etc/sysconfig/network-scripts/ifcfg-ens33   # 不是每个主机都是ens33
>    把ONBOOT=no，改为ONBOOT=yes
> 2. 重启系统
> 3. yum install net-tools
> 4. service network restart    // 重启网络

## 一、Docker

### 1.1 安装Docker

> - 安装依赖
>
>   > yum install -y yum-utils device-mapper-persistent-data lvm2
>
> - 设置镜像源仓库（阿里云）
>
> > yum-config-manager   --add-repo   http:**//**mirrors.aliyun.com**/**docker-ce**/**linux**/**centos**/**docker-ce.repo
>
> - 查看可用版本
>
> > yum list docker-ce --showduplicates | sort -r
>
> - 安装
>
> > yum -y install docker-ce-18.03.1.ce
>
> - 启动docker和设置开机自启
>
> > systemctl start docker 
> >
> > systemctl enable docker

### 1.2 安装MySQL

> - 拉取mysql
>
> > docker pull mysql
>
> ![image-20230303222403753](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230303222403753.png)
>
> - 创建挂载文件夹
>
> > 命令：cd /opt/
> > 命令：mkdir mysql_docker
> > 命令：cd mysql_docker/
> > 命令：echo $PWD
>
> - 创建实例并启动
>
> > docker run \
> > -p 3306:3306 \
> > --name heat-mysql \
> > --privileged=true \
> > --restart unless-stopped \
> > -v $PWD/conf:/etc/mysql/conf.d 
> > -v $PWD/logs:/logs 
> > -v $PWD/data:/var/lib/mysql 
> > -e MYSQL_ROOT_PASSWORD=123456
> > -d mysql \
> > --lower_case_table_names=1
>

> > -p 端口映射
> > --privileged=true  挂载文件权限设置
> > --restart unless-stopped  设置 开机后自动重启容器
> > -v $PWD/conf:/etc/mysql/conf.d   挂载配置文件
> > -v $PWD/logs:/logs     挂载日志
> > -v $PWD/data:/var/lib/mysql  挂载数据文件 持久化到主机
> > -e MYSQL_ROOT_PASSWORD=123456    设置密码
> > -d  mysql   后台启动,mysql
> > --lower_case_table_names=1	  让MySQL不区分大小写（0:大小写敏感;1:大小写不敏感
> >
> > ![image-20230303222929876](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230303222929876.png)
>
> - 进入mysql容器并登录
>
> > docker exec -it heat-mysql /bin/bash      (mysqlserver 是创建实例时的--name)
> > mysql -uroot -p
> > Enter password: 
>
> - 开启远程访问
>
> > select now();
> > use mysql;
> > select host,user from user;
> > ALTER USER 'root'@'%' IDENTIFIED WITH mysql_native_password BY '123456';
> > l
>
> ALTER USER 'root'@'localhost' IDENTIFIED WITH mysql_native_password BY '123456';

### 1.3安装redis

下载redis镜像

```sh
docker pull redis
```

创建配置文件

```sh
mkdir -p /mydata/redis/conf
touch /mydata/redis/conf/redis.conf
```

启动容器

```sh
docker run -p 6379:6379 --name redis \
-v /mydata/redis/data:/data \
-v /mydata/redis/conf/redis.conf:/etc/redis/redis.conf \
-d redis redis-server /etc/redis/redis.conf
```

运行redis

```sh
docker exec -it redis redis-cli
```

开启aof持久化

```sh
vi /mydata/redis/conf/redis.conf
# 添加如下内容
appendonly yes
```

重启redis

```sh
docker restart redis
```

设置自启动

> ```sh
> docker update redis --restart=always
> ```

## ubuntu18.04安装docker

1.更新软件源列表

sudo apt update

2.安装软件包依赖

sudo apt install apt-transport-https ca-certificates curl software-properties-common

3.在系统中添加[Docker](https://cloud.tencent.com/product/tke?from_column=20065&from=20065)的官方密钥

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

4.添加Docker源,选择stable长期稳定版

sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"

5.再次更新源列表

sudo apt update

6.查看可以安装的Docker版本

sudo apt-cache policy docker-ce

7.开始安装Docker（ce表示社区版）

sudo apt install docker-ce

9.查看安装的Docker版本

docker -v

10.启动Docker服务

sudo systemctl start docker//不好使换成service

11.设置开机自启动docker

sudo systemctl enable docker