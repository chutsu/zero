#!/usr/bin/env python3

f = open('zero/zero.c', 'r')

doc = []

group = ""

docstr_mode = False
docstr = []

keep_func = False
func_sig = []


def remove_comment_marker(string_block):
    retval = []
    for line in string_block.split('\n'):
        line = line.strip()
        if len(line) == 0:
            continue
        elif line[0:3] == "/**":
            continue
        elif line[0:2] == "*/":
            continue
        elif line[0] == "*":
            retval.append(line[1:].strip())

    return "\n".join(retval)


for line in f:
    if len(line.strip()) and line.strip()[0] == "#":
        continue

    if line.strip() == "/*":
        continue

    if line.strip() == "/** @} */":
        continue

    if line.strip() == "/**":
        docstr_mode = True
    if docstr_mode:
        docstr.append(line)
    if line.strip() == "*/" and docstr_mode:
        docstr_mode = False

        ds = "".join(docstr).rstrip()
        if ds.find("@defgroup") > 0:
            docstr.clear()
        elif ds.find("@ingroup") > 0:
            docstr.clear()
        else:
            keep_func = True
            continue

    ls = line.strip()
    if keep_func and len(ls) and ls[-1] not in ["{", "}", ",", ";"]:
        print("keep func off!")
        keep_func = False
        docstr.clear()
        func_sig.clear()

    elif keep_func and len(ls) and ls[-1] in ["{", "}", ",", ";"]:
        func_sig.append(line)
        if ls[-1] in [";", "{"]:
            doc_entry = {
                "docstr": remove_comment_marker("".join(docstr)),
                "func_sig": "".join(func_sig).replace("{", "").rstrip() + ";"
            }
            docstr.clear()
            func_sig.clear()
            keep_func = False
            docstr_mode = False
            doc.append(doc_entry)


f.close()

for docstr in doc:
    print("-" * 80)
    print("%s" % docstr["func_sig"])
    print()
    print("%s" % docstr["docstr"])
    print("-" * 80)

# print('\n'.join(doc))
