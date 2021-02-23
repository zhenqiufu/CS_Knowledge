GCC

## MAKE

make 命令依赖Makefile　或者 makefile文件来构建，也可以通过命令指定文件

```shell
make -f makerule.txt
make --file=makerule.txt
```

makefile文件编写

```makefile
<target> : <prerequisites> 
[tab]  <commands>
```

第一行，<target>为要生成的可执行文件，<prerequisites>为依赖

第二行，首先是一个[tab]，然后接的是命令<commands>．　

<target>不可省略，后两者可以省略，但二者存在一个

<target>不仅可以是可执行文件，也可以是命令，如常用的`make clean`

就使用了伪目标的概念，这里省略了依赖．

```makefile
clean:
      rm *.o
```

值得注意，如果文件中有clean同名文件，那么命令就不会执行，因为认为已经存在该文件，不需要构建，未避免误会，可以显示声明命令，这样就不会去检查clean了．

```makefile
.PHONY: clean
clean:
        rm *.o temp
```

当没有指定make具体对象，默认执行文件中的第一个目标．

```shell
make
```

依赖<prerequisites>是一组文件名，用空格相分隔．文件是否需要重建，主要根据依赖的时间戳．如果目标后没有<prerequisites>，则意味着与其他文件无关，只要目标存不存在，每次调用`make source.txt`都会生成．

```makefile
source.txt:
    echo "this is the source" > source.txt
```

如果需要生成多个文件，往往采用下面的写法。

> ```bash
> source: file1 file2 file3
> ```

上面代码中，source 是一个伪目标，只有三个前置文件，没有任何对应的命令。

> ```bash
> $ make source
> ```

执行`make source`命令后，就会一次性生成 file1，file2，file3 三个文件。这比下面的写法要方便很多。

> ```bash
> $ make file1
> $ make file2
> $ make file3
> ```



需要注意的是，每行命令在一个单独的shell中执行。这些Shell之间没有继承关系。

> ```bash
> var-lost:
>     export foo=bar
>     echo "foo=[$$foo]"
> ```

上面代码执行后（`make var-lost`），取不到foo的值。因为两行命令在两个不同的进程执行。一个解决办法是将两行命令写在一行，中间用分号分隔。

> ```bash
> var-kept:
>     export foo=bar; echo "foo=[$$foo]"
> ```

另一个解决办法是在换行符前加反斜杠转义。

> ```bash
> var-kept:
>     export foo=bar; \
>     echo "foo=[$$foo]"
> ```



正常情况下，make会打印每条命令，然后再执行，这就叫做回声（echoing）。命令前加上＠就不会打印．

**通配符**（wildcard）用来指定一组符合条件的文件名。Makefile 的通配符与 Bash 一致，主要有星号（*）、问号（？）和 [...] 。比如， *.o 表示所有后缀名为o的文件。

```bash
clean:
        rm -f *.o
```

**模式匹配**

Make命令允许对文件名，进行类似正则运算的匹配，主要用到的匹配符是%。比如，假定当前目录下有 f1.c 和 f2.c 两个源码文件，需要将它们编译为对应的对象文件。

> ```bash
> %.o: %.c
> ```

等同于下面的写法。

> ```bash
> f1.o: f1.c
> f2.o: f2.c
> ```

使用匹配符%，可以将大量同类型的文件，只用一条规则就完成构建。

**变量和赋值符**

Makefile 允许使用等号自定义变量。

> ```bash
> txt = Hello World
> test:
>     @echo $(txt)
> ```

上面代码中，变量 txt 等于 Hello World。调用时，变量需要放在 $( ) 之中。

调用Shell变量，需要在美元符号前，再加一个美元符号，这是因为Make命令会对美元符号转义。

> ```bash
> test:
>     @echo $$HOME
> ```

有时，变量的值可能指向另一个变量。

> ```bash
> v1 = $(v2)
> ```

上面代码中，变量 v1 的值是另一个变量 v2。



Makefile使用 Bash 语法，完成**判断和循环**。

> ```bash
> ifeq ($(CC),gcc)
>   libs=$(libs_for_gcc)
> else
>   libs=$(normal_libs)
> endif
> ```



> ```bash
> LIST = one two three
> all:
>     for i in $(LIST); do \
>         echo $$i; \
>     done
> 
> # 等同于
> 
> all:
>     for i in one two three; do \
>         echo $i; \
>     done
> ```

上面代码的运行结果。

> ```bash
> one
> two
> three
> ```





http://www.ruanyifeng.com/blog/2015/02/make.html

## CMAKE

autotools