import os
import subprocess

# Specify the directory to format code in
dir_path = ["src", 
            "cpp_examples"]

# Specify the path to the clang-format executable
clang_format_path = "clang-format-10"

# Specify the path to the stylesheet to use
style_path = ".clang-format"

# Get current directory
current_dir = os.path.dirname(os.path.realpath(__file__))

# Find all .c and .cpp files in the specified directory
files_to_format = []
for d in dir_path:
    for root, dirs, files in os.walk(d):
        root = os.path.join(current_dir, root)
        for file in files:
            if file.endswith((".c", ".cpp", ".h", ".hpp", ".cc", ".hh", ".cxx", ".hxx", ".C", ".H", ".cp", ".CPP", ".c++", ".h++")):
                files_to_format.append(os.path.join(root, file))

# print(files_to_format)
# Run clang-format on each file with the specified stylesheet
print("Formatting files...")
for file_path in files_to_format:
    subprocess.run([clang_format_path, "-i", "-style=file", "-fallback-style=none", file_path])

print("Done formatting!")