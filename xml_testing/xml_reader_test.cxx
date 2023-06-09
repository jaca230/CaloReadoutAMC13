#include <iostream>
#include <pugixml.hpp>

int main() {
    // Create an instance of pugi::xml_document
    pugi::xml_document doc;

    try {
        // Load and parse the XML file
        pugi::xml_parse_result result = doc.load_file("xml_test.xml");

        // Check if the parsing was successful
        if (!result) {
            std::cerr << "Error parsing XML file: " << result.description() << std::endl;
            return 1; // Return a non-zero value to indicate failure
        }

        // Access and print basic information from the XML file
        std::cout << "Root node name: " << doc.first_child().name() << std::endl;

        int numChildren = 0;
        for (pugi::xml_node childNode : doc.first_child().children()) {
            std::cout << "Child node name: " << childNode.name() << std::endl;
            std::cout << "Child node value: " << childNode.child_value() << std::endl;
            numChildren++;
        }

        std::cout << "Number of child nodes: " << numChildren << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1; // Return a non-zero value to indicate failure
    }

    return 0;
}
