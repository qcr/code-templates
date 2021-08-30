class PACKAGE_NAME__CAMEL:

    def __init__(self):
        print("Create instance of my 'PACKAGE_NAME__CAMEL' class")

    def dummy_method(self):
        print("Called dummy_method from 'PACKAGE_NAME__CAMEL' class")


def package_only_method():
    print("This method won't be available to users of your package unless "
          "it is imported in your __init__.py")
