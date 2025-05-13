target extended-remote:1337

define restart
  monitor reset halt
end


restart
break main

layout src
focus cmd
continue

