import subprocess

def get_children(lib):
    deps = dict()
    output = subprocess.getoutput('ldd ' + libs[0])
    lines = str(output).split('\n')
    for line in lines:
        # print(line)
        words = line.split("=>")

        if len(words) == 2:
            dep = words[0].replace(' ', '').replace('\t', '')
            location = words[1].split('(')[0].replace(' ', '').replace('\t', '')
            deps[dep] = location

    return deps

def get_dependencies_recursive(lib):
    print("Searching: ", lib)
    deps = get_children(lib)
    for dep in deps.keys():
        get_dependencies_recursive(deps[dep])


if __name__ == "__main__":
    libs = ['/lib/libgdal.so.30']

    get_dependencies_recursive(libs[0])

