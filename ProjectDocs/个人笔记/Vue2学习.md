

# Vue学习

## 安装

npm install -g @vue/cli@4.5.14

卸载

//卸载3.0之前的版本
npm uninstall -g vue-cli
//卸载3.0之后的版本（可以统一使用此指令卸载）
npm uninstall -g @vue/cli



npm config set registry https://registry.npm.taobao.org   配置淘宝镜像
npm config get registry   查看镜像地址是否设置成功

创建工程

vue create 项目名

运行项目

cd 项目文件夹

npm run serve

## 快速启动

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <!-- 1. 导入 vue 的脚本文件 -->
    <script src="https://unpkg.com/vue@next"></script>
  </head>
  <body>

    <!-- 2. 声明要被 vue 所控制的 DOM 区域 -->
    <div id="app">
      {{message}}
    </div>

    <!-- 3. 创建 vue 的实例对象 -->
    <script>
    const hello = {
        // 指定数据源，既MVVM中的Model
        data: function() {
            return {
                message: 'Hello Vue!'
            }
        }
    }
    const app = Vue.createApp(hello)
    app.mount('#app')
    </script>
  </body>
</html>

```

1.内容渲染指令

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Document</title>
  <script src="https://unpkg.com/vue@next"></script>
</head>
<body>
  <div id="app">
    <p>姓名：{{username}}</p>
    <p>性别：{{gender}}</p>

    <p>{{desc}}</p>
    <p v-html="desc"></p>
  </div>
  

  <script>
    const vm = {
      data: function(){ 
        return {
          username: 'zhangsan',
          gender: '男',
          desc: '<a href="http://www.baidu.com">百度</a>'
        }
      }
    }
    const app = Vue.createApp(vm)
    app.mount('#app')
  </script>
</body>
</html>
```

2.属性绑定指令  ,在属性前加上：

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@next"></script>
  </head>
  <body>
    <div id="app">
      <a :href="link">百度</a>
      <input type="text" :placeholder="inputValue">
      <img :src="imgSrc" :style="{width:w}" alt="">
    </div>

    <script>
      const vm = {
          data: function(){
            return {
              link:"http://www.baidu.com",
              // 文本框的占位符内容
              inputValue: '请输入内容',
              // 图片的 src 地址
              imgSrc: './images/demo.png',
              w:'500px'
          }
        }
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```

3.使用JavaScript表达式

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@3/dist/vue.global.js"></script>
  </head>
  <body>
    <!-- vue 实例要控制的 DOM 区域 -->
    <div id="app">
      <p>{{number + 1}}</p>
      <p>{{ok ? 'True' : 'False'}}</p>
      <p>{{message.split('').reverse().join('')}}</p>
      <p :id="'list-' + id">xxx</p>
      <p>{{user.name}}</p>
    </div>

    <script>
      const vm = {
        data: function(){
          return {
            number: 9,
            ok: false,
            message: 'ABC',
            id: 3,
            user: {
              name: 'zs',
            }
          }
        }
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```

4.事件绑定指令

> 两种方式

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@3/dist/vue.global.js"></script>
  </head>
  <body>
    <!-- vue 实例要控制的 DOM 区域 -->
    <div id="app">
      <h3>count 的值为: {{ count }} </h3>
      <button @click='count += 1'>+1</button>
      <button v-on:click="addCount">+1</button>  
    </div>

    <script>
      const vm = {
        data: function(){
          return {
            count:0,
          }
        },
        methods:{
            addCount(){
                this.count += 1
            }
        }
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```



5.条件渲染指令

> v-if和v-show的区别在于v-if在条件不成立时标签不会被创建

```html

<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@3/dist/vue.global.js"></script>
  </head>
  <body>
    <!-- vue 实例要控制的 DOM 区域 -->
    <div id="app">
      <button @click="flag = !flag">Toggle Flag</button>  
      <p v-if="flag">请求成功----被v-if控制</p>
      <p v-show="flag">请求成功---被v-show控制</p>
    </div>

    <script>
      const vm = {
        data: function(){
          return {
            flag:false,
          }
        }
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```

6.v-else和v-else-if指令

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@3/dist/vue.global.js"></script>
  </head>
  <body>
    <!-- vue 实例要控制的 DOM 区域 -->
    <div id="app">
        <p v-if="num > 0.5">随机数大于0.5</p>
        <p v-else>随机数小于等于0.5</p>
        <hr />
        <p v-if="type === 'A'">优秀</p>
        <p v-else-if="type === 'B'">良好</p>
        <p v-else-if="type === 'C'">一般</p>
        <p v-else>差</p>
        <div v-show="a">
            测试
        </div>
        <button @click="f">点击</button>
    </div>

    <script>
      const vm = {
        data: function(){
          return {
            num:Math.random(),
            type:"A",
            a : false
          }
        },
        methods:{
            f:function(){
                this.a = !this.a
            }
        }
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```

7.列表渲染指令

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@3/dist/vue.global.js"></script>
  </head>
  <body>
    <!-- vue 实例要控制的 DOM 区域 -->
    <div id="app">
      <ul>
          <li v-for="(user,i) in userList">
              索引是：{{i}},姓名是：{{user.name}}
          </li>
      </ul>
    </div>

    <script>
      const vm = {
        data: function(){
          return {
            userList:[
                {id:1,name:"zhangsan"},
                {id:2,name:"lisi"},
                {id:3,name:"wangwu"}

            ]
          }
        },
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```

8.v-for中的key

> key的作用是唯一化该标签，因为在for新建标签时vue的操作是重用之前的标签，那么在这之前，前一个标签如果发生了变化，那么这个变化会在新建的标签上

> v-model，是双向绑定，当input输入值时，绑定的变量也会变化

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Document</title>
    <script src="https://unpkg.com/vue@3/dist/vue.global.js"></script>
  </head>
  <body>
    <!-- vue 实例要控制的 DOM 区域 -->
    <div id="app">
        <!-- 添加用户的区域 -->
        <div>
            <input type="text" v-model="name">
            <button @click="addNewUser">添加</button>
        </div>

      <ul>
          <li v-for="(user,i) in userList" :key="user.id">
              <input type="checkbox">
              姓名：{{user.name}}
              
          </li>
      </ul>
    </div>

    <script>
      const vm = {
        data: function(){
          return {
            userList:[
                {id:1,name:"zhangsan"},
                {id:2,name:"lisi"},
                {id:3,name:"wangwu"}    

            ],
            name:'',
            nextId:3
          }
        },
        methods:{
            addNewUser(){
                this.userList.unshift({id:this.nextId,name:this.name})
                this.name='',
                this.nextId++
            }
        }
      }
      const app = Vue.createApp(vm)
      app.mount('#app')
    </script>
  </body>
</html>

```

## 组件化开发

定义自己的组件

Vue 中规定组件的后缀名是 .vue
每个 .vue 组件都由 3 部分构成，分别是

- template，组件的模板结构，可以包含HTML标签及其他的组件

- script，组件的 JavaScript 代码

- style，组件的样式

首先在components文件夹下创建一个.vue文件

```vue
<template>
    <p>hello</p>
</template>

<script>

</script>

<style scoped>

</style>

```

在App.vue的script标签中import刚才的组件,并注册，之后在template标签中使用

```vue
<script>
import hello from './components/hello.vue'
export default {
  name: 'App',
  components: {
    hello
  }
}
</script>
```

```vue
<template>
  <img alt="Vue logo" src="./assets/logo.png">
  <hello></hello>
</template>
```

### 组件间传值

> 组件可以由内部的Data提供数据，

通过在组件内部的script标签中使用export default

```vue
<template>
    <div>
        <h3>{{movie}}</h3>
    </div>
</template>

<script>
export default{
    setup() {
        
    },
    data:function(){
        return{
            movie:"金刚狼"
        }
    }
}
</script>

<style scoped>

</style>
```



> 也可以由父组件通过prop的方式传值。

```vue
<template>
    <div>
        <h3>{{title}}</h3>
    </div>
</template>

<script>
export default{
    setup() {
        
    },
    data:function(){
        return{
            movie:"金刚狼"
        }
    },
    props:["title"]
}
</script>

<style scoped>

</style>
```

这样在App.vue调用该组件时，就会有对应的props属性

```
<Movie :title="movie"></Movie>

  data:function(){
    return {
      movie:"金刚狼1"
    }
  }
```



> 兄弟组件之间可以通过Vuex等统一数据源提供数据共享。  

### Element-ui

Element是国内饿了么公司提供的一套开源前端框架，简洁优雅，提供了Vue、
React、Angular等多个版本。
文档地址：https://element.eleme.cn/#/zh-CN/
安装：npm i element-ui
引入 Element：  

```
import Vue from 'vue';
import ElementUI from 'element-ui';
import 'element-ui/lib/theme-chalk/index.css';
import App from './App.vue';

Vue.use(ElementUI);

new Vue({
  el: '#app',
  render: h => h(App)
});
```

然后就可以使用文档提供的组件进行开发了

### 图标库

由于Element UI提供的字体图符较少，一般会采用其他图表库，如著名的Font
Awesome
Font Awesome提供了675个可缩放的矢量图标，可以使用CSS所提供的所有特
性对它们进行更改，包括大小、颜色、阴影或者其他任何支持的效果。
文档地址：http://fontawesome.dashgame.com/
安装：npm install font-awesome
使用：import 'font-awesome/css/font-awesome.min.css'  

## axios

### 介绍

在实际项目开发中，前端页面所需要的数据往往需要从服务器端获取，这必然
涉及与服务器的通信。
Axios 是一个基于 promise 网络请求库，作用于node.js 和浏览器中。
Axios 在浏览器端使用XMLHttpRequests发送网络请求，并能自动完成JSON
数据的转换 。
安装：npm install axios
地址：https://www.axios-http.cn/  

在实际项目开发中，几乎每个组件中都会用到 axios 发起数据请求。此时会遇
到如下两个问题：

- 每个组件中都需要导入 axios

- 每次发请求都需要填写完整的请求路径

可以通过全局配置的方式解决上述问题：  

```jsx
import axios from 'axios'

// 配置基本URL
axios.defaults.baseURL = "http://localhost:8088";
// 配置全局axios
// app.config.globalProperties.$http = axios    //vue3
Vue.prototype.$http = axios //vue2
```

### 在组件中使用axios

```xml
// 向给定ID的用户发起请求
this.$http.get('/user?ID=12345')
  .then(function (response) {
    // 处理成功情况
    console.log(response);
  })
  .catch(function (error) {
    // 处理错误情况
    console.log(error);
  })
  .then(function () {
    // 总是会执行
  });

// 上述请求也可以按以下方式完成（可选）
this.$http.get('/user', {
    params: {
      ID: 12345
    }
  })
  .then(function (response) {
    console.log(response);
  })
  .catch(function (error) {
    console.log(error);
  })
  .then(function () {
    // 总是会执行
  });  

// 支持async/await用法
async function getUser() {
  try {
    const response = await this.$http.get('/user?ID=12345');
    console.log(response);
  } catch (error) {
    console.error(error);
  }
}
```

axios通常放在created中，在页面开始加载之前执行，但执行过程是异步的

```jsx
    created:function(){
        this.$http.get('/user/findAll')
            .then((response)=>{
                // 处理成功情况
                this.tableData = response.data.records;
                console.log(response);
                
            })
        }
```

> 这里=>的作用是为了解决好作用域的问题，因为tableData是在data中定义的变量

### 跨域问题

> 因前端访问后端接口存在跨域问题，所以需在后端进行相应配置允许前端访问

1. 添加配置文件

![image-20230405191302581](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230405191302581.png)

2. 在Controller类上添加@CrossOrigin注解，这样这个类中的所有接口都可以被前端访问

## vue router

Vue路由vue-router是官方的路由插件，能够轻松的管理 SPA 项目中组件的切换。 Vue的单页面应用是基于路由和组件的，路由用于设定访问路径，并将路径和组件映射起来

vue-router 目前有 3.x 的版本和 4.x 的版本，`vue-router 3.x 只能结合 vue2 进行使用，vue-router 4.x 只能结合 vue3 进行使用`

安装：`npm install vue-router@3`/4

### 快速上手

在`router/index.js`初始化router

```jsx
import VueRouter from "vue-router";
import Vue from "vue";
import Book from "@/components/Book";
import Music from "@/components/Music";

// 将VueRouter设置为Vue的插件
Vue.use(VueRouter)

const router = new VueRouter({
    routes: [
        {path:'/book', component: Book},
        {path:'/music', component: Music},
    ]
})

export default router
```

在`main.js`中导入router

```jsx
import router from "@/router";
new Vue({
  render: h => h(App),
  router: router,
}).$mount('#app')
```

在其他文件（一般为App.vue）中使用router

```jsx
<template>
	<!--声明路由链接-->
	<router-link to="/book">书</router-link>
	<router-link to="/music">音乐</router-link>
	<!--声明路由占位符标签-->
	<router-view></router-view>
</template>
```

### 嵌套路由

只需要在一个路由后面加上children即可。

```jsx
{path: '/book', component: Book, children: [
    {path: 'java', component: Java},
    {path: 'python', component: Python},
]},
```

Book组件中，必须也要有`router-view`

```jsx
<template>
  <div>
  <h1>我是Book</h1>
    <router-link to="/book/java">java</router-link>
    <router-link to="/book/python">python</router-link>
    <router-view></router-view>
  </div>
</template>
```

如果没有的话，一个router-view是不可以渲染两个组件的，导致只能看到父亲组件

### 动态路由

> 有这样一个需求，三个路由/music/1、/music/2、/music/3，让他们指向同一个组件，并在组件中获取1，2，3

就比如`/image/1`这样的路由，1是图片的id

```jsx
<template>
    <div>
        <h3>音乐</h3>
        <router-link to="/music/1">青花瓷</router-link>
        <router-link to="/music/2">兰亭序</router-link>
        <router-view></router-view>
    </div>
</template>
```

router定义

```jsx
        {
            path:'/music', 
            component: Music,
            children:[{path:':id',component:MyMusic,},],            
        },
```

组件中拿到id

```jsx
<template>
    <div>
        <h3>我的第{{ $route.params.id }}首音乐</h3>
    </div>
</template>
```

开启prop传递参数，方便我们更好的拿到参数

```jsx
{
    path:'/music', 
    component: Music,
    children:[{path:':id',component:MyMusic,props:true},],            
},
```

```
<template>
    <div>
        <h3>我的第{{ id }}首音乐</h3>
    </div>
</template>

<script>
export default {
    props:["id"],
}
</script>
```

![image-20230406193024924](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20230406193024924.png)

# VueX

对于组件化开发来说，大型应用的状态往往跨越多个组件。在多层嵌套的父子组件之间传递状态已经十分麻烦，而Vue更是没有为兄弟组件提供直接共享数据的办法。

每一个 Vuex 应用的核心就是 store（仓库）。“store”基本上就是一个容器，它包含着你的应用中大部分的**状态 (state)**

- Vue2对应VueX3
- Vue3对应VueX4

安装：`npm install vuex@3 --save`

![img](https://thexb.notion.site/image/https%3A%2F%2Fs3-us-west-2.amazonaws.com%2Fsecure.notion-static.com%2F221c1ad6-64d6-450d-8c70-1037578dddee%2FUntitled.png?id=28d5220c-d0fc-42ca-967e-f873cd65a7f1&table=block&spaceId=7b56dfa1-f700-49b2-a3c7-5b08ec34c87a&width=2000&userId=&cache=v2)

在`store/index.js`初始化vuex

```jsx
import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

const store = new Vuex.Store({
  state: {
    count: 0
  },
  mutations: {
    increment (state) {
      state.count++
    }
  }
})
```

在`main.js`导入vuex

```jsx
import Vue from 'vue'
import App from './App.vue'
import store from "@/store";

Vue.config.productionTip = false

new Vue({
  render: h => h(App),
  store: store,
}).$mount('#app')
```

在组件中使用

```jsx
<template>
  <div id="app">
    {{ this.$store.state.count }}
    <button @click="add">加一</button>
  </div>
</template>

<script>
export default {
  name: 'App',
  methods: {
    add() {
      this.$store.commit("increment")
    }
  }
}
</script>
```

简化this.$store.state

```jsx
<template>
  <div id="app">
    {{ count }}
    <button @click="add">加一</button>
  </div>
</template>

<script>
import {mapState} from 'vuex';

export default {
  name: 'App',
  computed: {
    ...mapState([
     // 将this.$store.state.count 映射为 count
     'count',
   ])
  },
}
</script>
```

`对于一个列表，我们可以只想要获取其中done属性为true的元素，因此我们需要设置getter`，通过`mapGetter`简化getter

```jsx
import Vue from 'vue'
import Vuex from 'vuex'

Vue.use(Vuex)

const store = new Vuex.Store({
    state: {
        todos: [
            {id: 1, text: '...', done: true},
            {id: 2, text: '...', done: false}
        ]
    },
    getters: {
        doneTodos: state => {
            return state.todos.filter(todo => todo.done)
        }
    },
})

export default store
<template>
  <div id="app">
    {{ doneTodos }}
    <button @click="add">加一</button>
  </div>
</template>

<script>
import {mapState, mapGetters} from 'vuex';

export default {
  name: 'App',
  computed: {
    ...mapGetters([
        // 将this.$store.getter.doneTodos 映射为 doneTodos
        'doneTodos',
    ])
  },
}
</script>
```

简化Mutations

```jsx
import { mapMutations } from 'vuex'

methods: {
    ...mapMutations([
			 // 将 `this.increment()` 映射为 `this.$store.commit('increment')`
      'increment',
       // `mapMutations` 也支持载荷：
      'incrementBy' // 将 `this.incrementBy(amount)` 映射为 `this.$store.commit('incrementBy', amount)`
    ]),
}
```

# MockJs

Mock.js 是一款前端开发中拦截Ajax请求再生成随机数据响应的工具，可以用来模拟服务器响应. 优点是非常简单方便, 无侵入性, 基本覆盖常用的接口数据类型.。 支持生成随机的文本、数字、布尔值、日期、邮箱、链接、图片、颜色等。

安装：`npm install mockjs`

在项目中`创建mock目录，新建index.js文件`

```jsx
import Mock from "mockjs"

Mock.mock('/search', {
    "ret": 0,
    "data": {
        // 生成随机日期
        "mtime": "@datetime",
        // 生成随机数组1-800
        "score|1-800": 1,
        // 生成随机中文名字
        "nickname": "@cname",
        // 生成图标
        "img": "@image('200x100', '#ffcc33', '#FFF', 'png', 'Github')"
    }
})
```

在`main.js`中导入mock

```jsx
import "./mock";
```

拦截`/search?id=1`这样带参数的路由，需要在拦截的地方，加上正则表达式

```jsx
import Mock from "mockjs"

Mock.mock(RegExp('/search.*'), {
    "ret": 0,
    "data": {
    }
})
```

> 注意：使用mockjs时不要在main.js里配置baseURL

官网生成文档：

[Syntax Specification · nuysoft/Mock Wiki](https://github.com/nuysoft/Mock/wiki/Syntax-Specification#数据模板定义规范-dtd)

