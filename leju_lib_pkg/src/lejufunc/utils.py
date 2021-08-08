#coding=utf-8

def is_number(number):
    try:
        return float(number)
    except:
        return None

def judge_range_number(number, min, max):
    number = is_number(number)
    if number is not None:
        if min <= number <= max:
            return number
    raise ValueError("参数错误, 应为数字类型, 范围为[{}, {}]".format(min, max))





