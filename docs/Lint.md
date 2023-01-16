# Tester sa production

## En utilisant des linters avec un package Ros 2

`linter` pour les code cpp : permet de vérifier si le code écrit respecte bien la _norme de style_ 

### Set-up du package

Faire apparaître les dépendance dans *package.xml*

[Liste des ament_lint_common]( https://index.ros.org/p/ament_lint_common/)

```xml
<test_depend>ament_lint_auto</test_depend>

<!-- this recursively depends on a set of common linters (pour voir la liste voir le lien ci-dessus) -->
<test_depend>ament_lint_common</test_depend>
<!-- ou si on veut en set-up un par un -->
<test_depend>ament_cmake_cpplint</test_depend>
<test_depend>ament_cmake_pycodestyle</test_depend>

```

Ajouter l'option de test dans *CMakelist.txt*

```cmake
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Pour un package spécifique
if(BUILD_TESTING)
  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(MAX_LINE_LENGTH 120)
endif()
```

Compiler le package

```bash
colcon build # --select-package
```

Jouer les tests ([Documentation pour *colcon test*](https://colcon.readthedocs.io/en/released/reference/verb/test.html))

```bash
colcon test
colcon test --event-handlers=console_cohesion+ --return-code-on-test-failure --packages-select tennis_court
# pour voir toutes les options :
colcon test --help # ou lire la doc
```

### Différents linters et autres checkers

### ament_lint_auto

[docs](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst)

### ament_lint_common

[docs](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst)

### ament_lint

https://github.com/google/styleguide)

### Add linter in Ros 2

[Tuto non testé](https://ubuntu.com/blog/how-to-add-a-linter-to-ros-2)

