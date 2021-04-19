# QCR Code Templates

This repository defines shared templates for commonly performed actions within the [QUT Centre for Robotics (QCR)](https://research.qut.edu.au/qcr). We've made this project public as most of the templates have a general use case, and aren't directly tied to QCR.

Templates can be used through a single script, and new templates are created by writing some basic template script in a new folder. The template 'engine' is ~250 lines of (admittedly terse) Bash.

## How to use a template

Place the script in your path somewhere (`~/bin/` is suggested), and make it executable:

```
wget https://github.com/qcr/code_templates/raw/master/qcr_templates && chmod a+x qcr_templates
```

The intention is once you've got the above script, it will always give you access to the latest templates (no need to update).

Your new projects can be created from a template simply by making a new folder and running the script with your chosen template inside that folder. For example:

```
qcr_templates ros_package
```

This will retrieve the template, and start a prompt asking you for values for your project. In general, it's best to use [snake_case](https://en.wikipedia.org/wiki/Snake_case) for programming variable values (i.e. `my_variable_value` not `myVariableValue` as our modification function assumes snake_case).

## How templates work

We use a very basic custom templating method in this project, with templates being declared by creating a new folder in this repository. Templates are defined using named variables, the user is prompted at runtime for values for these variables, and then a project is created from the template with the runtime values applied. Variable values can be used to:

- replace in-file values in code / text
- conditionally include blocks of code / text in files
- generate filenames based on variables
- conditionally create files

Template variable names are typically upper snake case (i.e. `MY_VARIABLE`), can have default values which will be shown in the prompt, and are evaluated using Bash. This means that any variable with no value is considered false, and all other values considered true. A current limitation is that variables with default values cannot be changed to have no value by the user at runtime.

Variables are declared in a special file called `.variables.yaml` at the root of each template, with their syntax described [below](#creating-your-own-templates).

### In-file text replacement

Variables are replaced in text using their runtime value, with the `__CAMEL` and `__PASCAL` modifiers supported. For example, the following Python template:

```python

class MY_VARIABLE__PASCAL:

  def __init__(self):
    self._MY_VARIABLE = None

def MY_VARIABLE__CAMEL():
  print("Hi")
```

when given `MY_VARIABLE='obstacle_detector'`, would produce:

```python

class ObstacleDetector:

  def __init__(self):
    self.obstacle_detector = None

def obstacleDetector():
  print("Hi")
```

### Conditional in-file blocks

Variables can also be used to declare whether blocks of code should be included in the output. Blocks begin with a `TEMPLATE_START variable_1 variable_2 ...` line, and end with a `TEMPLATE_END` line. The block is included if _any_ of `variable_1 variable_2 ...` have a value, and will only be excluded if _all_ are empty. For example, the following CMake template:

```cmake

catkin_package(
  TEMPLATE_START ADD_MSGS ADD_SERVICES ADD_ACTIONS
  CATKIN_DEPENDS message_runtime
  TEMPLATE_END
  )
```

includes a dependency on `message_runtime` if any of `ADD_MSGS`, `ADD_SERVICES`, `ADD_ACTIONS` have a value. The `TEMPLATE_*` lines are removed from the result, with the output being:

```cmake
catkin_package(
  CATKIN_DEPENDS message_runtime
  )
```

The opposite relationship (include if _all_ have a value) isn't yet supported, but should be supported in the future.

### Variable file names

File names can be given variable values simply by using the variable name in the filename. For example, a file called `MY_VARIABLE.cpp` with a runtime value of `MY_VARIABLE='object_detector'` would be renamed to `object_detector.cpp`.

The `__CAMEL` and `__PASCAL` modifiers aren't currently supported, but can be in the future if needed.

### Conditional file existence

Another special file called `.files.yaml` marks files which should only exist under certain conditions. It's syntax is based on very basic key-value pairs (`filename: variable_1 variable_2 ...`), with the filed included if _any_ of `variable_1 variable_2 ...` have a value. See existing templates for examples.

## Creating your own templates

Creating your own templates is almost as simple as using templates. To create your own template:

1. Clone this repository locally:

   ```
   git clone https://github.com/qcr/code_templates
   ```

2. Make a new folder with the name of your template. For example, a template called `my_new_template` is denoted by a folder called `my_new_template`.

3. Create a `.variables.yaml` file in your new folder. The format is the following:

   ```yaml
   VARIABLE_NAME:
     text: "Text to be displayed to user in prompt"
     default: "Default static value"
   VARIABLE_WITH_DYNAMIC_DEFAULT:
     text: "Variable with default value determined at runtime"
     default: $(echo "This Bash code will be executed")
   OPTIONAL_VARIABLE:
     text: "Variable will be left blank if the user provides no input"
     default: ""
   ```

4. Create the files for your template, taking advantage of whichever [variable features](#how-templates-work) your template requires.

5. **Test** your template locally before pushing to master (as soon as it's pushed everyone can use it). Test locally by directly running the `use_template` script with local files instead of the remote:

   ```
   LOCAL_LOCATION=/path/to/local/clone/of/this/repo ./use_template my_new_template
   ```

6. Once it works, push to the master branch. Done!

Please note: a very crude YAML parser is written in [`use_template`](./use_template) to keep the dependencies of this software as low as possible. I emphasise, _crude_. You should not expect full YAML functionality (keep values on same line as key, don't use line breaks, no escape characters, etc.).
