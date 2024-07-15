#include <iostream>
#include <fstream>

int main() {
    // コピー元とコピー先のファイル名
    std::string sourceFilename = "config/param1.yaml";
    std::string destFilename = ".yml";

    try {
        // 入力ファイルをバイナリモードで開く
        std::ifstream sourceFile(sourceFilename, std::ios::binary);
        if (!sourceFile) {
            throw std::runtime_error("入力ファイルを開けませんでした");
        }

        // 出力ファイルをバイナリモードで開く
        std::ofstream destFile(destFilename, std::ios::binary);
        if (!destFile) {
            throw std::runtime_error("出力ファイルを開けませんでした");
        }

        // 入力ファイルからデータを読み取り、出力ファイルに書き込む
        destFile << sourceFile.rdbuf();

        // ファイルを閉じる
        sourceFile.close();
        destFile.close();

        std::cout << "ファイルが正常にコピーされました。\n";
    } catch (const std::exception& e) {
        std::cerr << "エラー: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}