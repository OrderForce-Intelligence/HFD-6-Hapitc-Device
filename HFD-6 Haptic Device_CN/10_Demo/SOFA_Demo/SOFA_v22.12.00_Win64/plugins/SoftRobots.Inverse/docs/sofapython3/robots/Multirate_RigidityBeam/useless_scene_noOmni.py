import Sofa
import controller_withProxy

# Scene creation method
def createScene(rootNode):
    rootNode.addObject('PythonScriptController', name='me', filename='controller_withProxy_noOmni.py', classname='Controller')
    return 0 
