#include <iostream>
#include <fstream>
#include <string>
#include <vector>

bool check_gbk(unsigned char b1, unsigned char b2) {
    if (b1 >= 0x81 && b1 <= 0xFE) {
        if ((b2 >= 0x40 && b2 <= 0x7E) || (b2 >= 0x80 && b2 <= 0xFE)) {
            return true;
        }
    }
    return false;
}

int main() {
    //请在这里替换为需要检查的文件的路径 (注意双斜杠)
    std::string path = "D:\\Samp\\Rich_CV\\esp32\\Common_eps32.cpp";

    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        std::cout << "Error: Cannot open file." << std::endl;
        return 1;
    }

    std::vector<unsigned char> buf((std::istreambuf_iterator<char>(file)),
        std::istreambuf_iterator<char>());
    file.close();

    size_t line = 1;
    size_t col = 1;
    size_t i = 0;
    bool err = false;

    while (i < buf.size()) 
    {
        unsigned char b1 = buf[i];

        if (b1 == '\n') {
            line++;
            col = 1;
            i++;
            continue;
        }

        if (b1 <= 0x7F) {
            i++;
            col++;
            continue;
        }

        if (i + 1 < buf.size()) {
            unsigned char b2 = buf[i + 1];

            if (check_gbk(b1, b2)) {
                i += 2;
                col += 2;
            }
            else {
                // 精准打印出问题的行号和列号
                std::cout << "Illegal character found at Line: " << line
                    << ", Column: " << col << std::endl;
                err = true;
                i++;
                col++;
            }
        }
        else {
            std::cout << "Illegal character at EOF, Line: " << line << std::endl;
            err = true;
            i++;
        }
    }

    if (!err) {
        std::cout << "Check finished. All characters are valid ANSI/GBK." << std::endl;
    }

    return 0;
}
