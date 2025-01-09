# MyBatis

## 一、目录结构

```
├─main
│  ├─java
│  │  └─com
│  │      └─heat
│  │          └─system
│  │              ├─controller
│  │              │      ***Controller.java
│  │              │
│  │              ├─domain
│  │              │      ***.java
│  │              │
│  │              ├─mapper
│  │              │      ***Mapper.java
│  │              │
│  │              └─service
│  │                  │  I***Service.java
│  │                  │
│  │                  └─impl
│  │                          ***ServiceImpl.java
│  │
│  └─resources
│      └─mapper
│          └─system
│                  ***Mapper.xml
```



## 二、文件说明

- __***Controller.java__

> 前端控制接口，调用Service接口实现对数据库的操作

- __***.java__

> 对象

- __***Mapper.java__

> 映射接口，映射XML与Service

- __***Service.java__

> Service接口

- __***ServiceImpl.java__

> Service业务层处理，对Service中定义的业务做具体处理

- __***Mapper.xml__

> SQL文件

## 三、文件基本结构

- __***Controller.java__

```java
@RestController
@RequestMapping("/basic/thermocouple")
public class BasicThermocoupleController extends BaseController
{
    @Autowired
    private IBasicThermocoupleService basicThermocoupleService;

    @Autowired
    private IBasicFurnaceThermocoupleService basicFurnaceThermocoupleService;

    /**
     * 查询电偶信息列表
     */
    @PreAuthorize("@ss.hasPermi('system:thermocouple:list')")
    @GetMapping("/list")
    public TableDataInfo list(BasicThermocouple basicThermocouple)
    {
        startPage();
        List<BasicThermocouple> list = basicThermocoupleService.selectBasicThermocoupleList(basicThermocouple);
        return getDataTable(list);
    }
}

```

- __***.java__

```java
public class BasicThermocouple extends BaseEntity
{
    /**定义数据表的对象*/
    
    /** 电偶id */
    private Long thermocoupleId;

    /** 使用区域 */
    @Excel(name = "使用区域")
    private String thermocouplePosition;
    
    /**下面是定义的一些set和get方法

    public void setThermocoupleId(Long thermocoupleId) 
    {
        this.thermocoupleId = thermocoupleId;
    }
}

```

- __***Mapper.java__

```java
public interface BasicThermocoupleMapper 
{
    /**定义服务映射，Service中定义的接口都需要在这里映射*/

    /**
     * 查询传感器信息列表
     * 
     * @param basicThermocouple 传感器信息
     * @return 传感器信息集合
     */
    public List<BasicThermocouple> selectBasicThermocoupleList(BasicThermocouple basicThermocouple);
}

```

- __***Service.java__

```java
public interface IBasicThermocoupleService 
{
    /** 定义服务接口*/ 
    
    /**
     * 查询传感器信息
     * 
     * @param thermocoupleId 传感器信息主键
     * @return 传感器信息
     */
    public BasicThermocouple selectBasicThermocoupleByThermocoupleId(Long thermocoupleId);
}

```

- __***ServiceImpl.java__

```java
@Service
public class BasicThermocoupleServiceImpl implements IBasicThermocoupleService 
{
    @Autowired
    private BasicThermocoupleMapper basicThermocoupleMapper;
    
    /**Service的具体业务*/

    /**
     * 查询传感器信息
     * 
     * @param thermocoupleId 传感器信息主键
     * @return 传感器信息
     */
    @Override
    public BasicThermocouple selectBasicThermocoupleByThermocoupleId(Long thermocoupleId)
    {
        return basicThermocoupleMapper.selectBasicThermocoupleByThermocoupleId(thermocoupleId);
    }
}

```

- __***Mapper.xml__

```html
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE mapper
PUBLIC "-//mybatis.org//DTD Mapper 3.0//EN"
"http://mybatis.org/dtd/mybatis-3-mapper.dtd">
<mapper namespace="com.heat.system.mapper.BasicThermocoupleMapper">
    <resultMap type="BasicThermocouple" id="BasicThermocoupleResult">
        <result property="thermocoupleId"    column="thermocouple_id"    />
        <result property="thermocoupleStandard"    column="thermocouple_standard"    />
    </resultMap>

    <sql id="selectBasicThermocoupleVo">
        select thermocouple_id, thermocouple_standard, 
    </sql>
    <select id="selectBasicThermocoupleByThermocoupleId" parameterType="Long" resultMap="BasicThermocoupleResult">
        <include refid="selectBasicThermocoupleVo"/>
        where thermocouple_id = #{thermocoupleId}
    </select>
</mapper>
```

## 四、ResultType和ResultMap

> 两者的作用都是描述如何从数据库结果集中加载对象。

> ResultType:  多用于简单查询，会自动创建ResulltMap，并将数据库结果映射到里面

> ResultMap：多用于复杂查询association、collection

### 4.1 Result Type的用法

```java
// 创建对象类
// -------teacher.java
public class teacher(){
    private Long tId;
    private String tName;    
}
// -------teacherMapper.xml
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE mapper
PUBLIC "-//mybatis.org//DTD Mapper 3.0//EN"
"http://mybatis.org/dtd/mybatis-3-mapper.dtd">
<mapper namespace="com.heat.system.mapper.BasicThermocoupleMapper">
```

