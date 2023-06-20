#include <iostream>
#include <string>
#include <vector>
#include <pugixml.hpp>

std::vector<std::string> generateStringList(const std::string& xmlFilePath, int frontendIndex, int maxIterations) {
    std::vector<std::string> stringList;

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(xmlFilePath.c_str());

    if (!result) {
        std::cerr << "Error parsing XML file: " << result.description() << std::endl;
        return stringList;
    }

    pugi::xml_node frontend = doc.child("frontend");
    int frontendIterations = 0;

    while (frontend && frontendIterations < maxIterations) {
        int id = frontend.attribute("id").as_int();

        if (id == frontendIndex) {
            break;
        }

        frontend = frontend.next_sibling("frontend");
        frontendIterations++;
    }

    if (!frontend) {
        std::cout << "Frontend with the specified index not found." << std::endl;
        return stringList;
    }

    pugi::xml_node slot = frontend.child("slot");
    int slotIterations = 0;

    while (slot && slotIterations < maxIterations) {
        int slotId = slot.attribute("id").as_int();
        std::string slotType = slot.attribute("type").as_string();
        std::string slotString;

        if (slotType == "FC7") {
            slotString = "FC7-";
            if (slotId < 10) {
                slotString += "0";
            }
            slotString += std::to_string(slotId);
        }
        else if (slotType == "Rider" || slotType == "WFD") {
            slotString = "Rider";
            if (slotId < 10) {
                slotString += "0";
            }
            slotString += std::to_string(slotId);
        }

        stringList.push_back(slotString);

        slot = slot.next_sibling("slot");
        slotIterations++;
    }

    return stringList;
}

int main() {
    std::string xmlFilePath = "xml_test.xml";
    int frontendIndex = 0;
    int maxIterations = 1000;

    std::vector<std::string> result = generateStringList(xmlFilePath, frontendIndex, maxIterations);

    if (result.empty()) {
        std::cout << "No slots found for the specified frontend index." << std::endl;
    }
    else {
        std::cout << "Generated string list:" << std::endl;
        for (const std::string& str : result) {
            std::cout << str << std::endl;
        }
    }

    return 0;
}
