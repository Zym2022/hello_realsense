import ctypes
import threading
import time

class Var2Py(ctypes.Structure):
    _fields_ = [
        ('best_fitness_score', ctypes.c_float),

        ('rotation_00', ctypes.c_float),
        ('rotation_01', ctypes.c_float),
        ('rotation_02', ctypes.c_float),
        ('rotation_10', ctypes.c_float),
        ('rotation_11', ctypes.c_float),
        ('rotation_12', ctypes.c_float),
        ('rotation_20', ctypes.c_float),
        ('rotation_21', ctypes.c_float),
        ('rotation_22', ctypes.c_float),

        ('translation_x', ctypes.c_float),
        ('translation_y', ctypes.c_float),
        ('translation_z', ctypes.c_float)
    ]

class Py2Var(ctypes.Structure):
    _fields_ = [
        ('camera_frame_width', ctypes.c_int),
        ('camera_frame_height', ctypes.c_int),
        ('camera_fps', ctypes.c_int),

        ('template_list_path', ctypes.c_char_p),

        ('not_from_bag', ctypes.c_bool),
        ('bag_path', ctypes.c_char_p)
    ]

class myThread (threading.Thread):   #继承父类threading.Thread
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):                   #把要执行的代码写到run函数里面 线程在创建后会直接运行run函数 
        print("Start Threading to run templete alignment\n")
        so.main()



if __name__ == '__main__':
    so = ctypes.CDLL("/home/zju/realsense_ws/build/libhello-realsense.so")
    py2var = Py2Var(848, 480, 30, b"/home/zju/realsense_ws/template_pcd/template_list.txt", False, b"/home/zju/Documents/405.bag")
    so.set_py2var.argtypes = [ctypes.POINTER(Py2Var)]
    so.set_py2var(ctypes.byref(py2var))

    thread1 = myThread()
    thread1.start()

    so.get_var2py.restype = ctypes.POINTER(Var2Py)


    while True:
        var2py_ptr = so.get_var2py()
        var2py = var2py_ptr.contents
        print("\n")
        print(f"from python---best score: {round(var2py.best_fitness_score, 6)}\n")
        print(f"from python---translation: x={round(var2py.translation_x,3)}, y={round(var2py.translation_y,3)}, z={round(var2py.translation_z,3)}\n")
        time.sleep(5)

