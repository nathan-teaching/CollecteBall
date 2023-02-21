# Uniformiser son code

Lorsqu'on développe à plusieurs, il est intéressant de choisir une norme et un style pour la production des codes.
Afin de s'assurer que ces styles sont respectés, on peut avoir recours à des "linter".

## Utiliser `colcon test` pour tester son code

### Set-up du package

Faire apparaître les dépendance dans *package.xml*

[Liste des ament\_lint\_common]( https://index.ros.org/p/ament_lint_common/)

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

### Différents linters
- Voir la [liste de ament](https://github.com/ament/ament_lint) 
- Créer/ajouter un linter custom : [Tuto non testé](https://ubuntu.com/blog/how-to-add-a-linter-to-ros-2)
- Exemple : [Google Style Guide](https://google.github.io/styleguide/), [flake8](https://flake8.pycqa.org/en/latest/)
