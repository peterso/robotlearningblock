#!/usr/bin/env python3
import os
import subprocess
import re
import csscompressor
import htmlmin
import jsmin
import tempfile
from bs4 import BeautifulSoup

def minify_content(content):
    """
    Minify HTML content including embedded CSS and JavaScript
    """
    # Parse HTML
    soup = BeautifulSoup(content, 'html.parser')

    # Minify embedded JavaScript
    for script in soup.find_all('script'):
        if script.string and not script.get('src'):  # Only minify inline scripts
            try:
                minified_js = jsmin.jsmin(script.string)
                script.string = minified_js
            except Exception as e:
                print(f"Warning: Could not minify JavaScript: {e}")

    # Minify embedded CSS
    for style in soup.find_all('style'):
        if style.string:
            try:
                minified_css = csscompressor.compress(style.string)
                style.string = minified_css
            except Exception as e:
                print(f"Warning: Could not minify CSS: {e}")

    # Convert back to string and minify HTML
    try:
        content = str(soup)
        # Minify the entire HTML
        content = htmlmin.minify(content,
                               remove_comments=True,
                               remove_empty_space=True,
                               remove_all_empty_space=True,
                               reduce_boolean_attributes=True)
    except Exception as e:
        print(f"Warning: Could not minify HTML: {e}")

    return content

def sanitize_variable_name(name):
    """
    Convert a filename to a valid C variable name
    """
    # Replace non-alphanumeric characters with underscore
    name = re.sub(r'[^a-zA-Z0-9]', '_', name)
    # Ensure it doesn't start with a number
    if name[0].isdigit():
        name = f"_{name}"
    return name

def convert_file(html_file):
    """
    Convert HTML file to C header file with minification
    """
    # Get base name without extension
    base_name = os.path.splitext(html_file)[0]
    header_file = f"web_{base_name}.h"

    try:
        # Read and minify content
        with open(html_file, 'r', encoding='utf-8') as f:
            content = f.read()

        minified_content = minify_content(content)

        # # Save minified file to .html.min
        # minified_file = f"{base_name}_min.html"
        # with open(minified_file, 'w', encoding='utf-8') as f:
        #     f.write(minified_content)

        # Create a temporary file for the minified content
        with tempfile.NamedTemporaryFile(mode='w', suffix='.html', delete=False, encoding='utf-8') as temp_f:
            temp_f.write(minified_content)
            temp_file_path = temp_f.name

        # Run xxd command on the temporary file
        xxd_output = subprocess.check_output(['xxd', '-i', temp_file_path]).decode('utf-8')

        # Process the xxd output
        lines = xxd_output.split('\n')
        processed_lines = []

        # Generate sanitized variable name
        var_name = sanitize_variable_name(base_name)

        for line in lines:
            if line.strip():
                # Replace the temporary filename with our sanitized variable name
                line = re.sub(r'_tmp_[a-zA-Z0-9_]+_html', f"_{var_name}_html", line)

                # Add const to the array declaration
                if '[' in line and 'unsigned' in line:
                    line = line.replace('unsigned char', 'const unsigned char')

                # Add const to the length variable
                elif 'unsigned long' in line:
                    line = line.replace('unsigned long', 'const unsigned long')

                processed_lines.append(line)

        # Write the processed output to the header file
        with open(header_file, 'w') as f:
            # Add header guard
            guard_name = f"{var_name.upper()}_H"
            f.write(f"#ifndef {guard_name}\n")
            f.write(f"#define {guard_name}\n\n")

            # Add file size comment
            original_size = len(content)
            minified_size = len(minified_content)
            savings = ((original_size - minified_size) / original_size) * 100

            f.write(f"/*\n")
            f.write(f" * Original size: {original_size:,} bytes\n")
            f.write(f" * Minified size: {minified_size:,} bytes\n")
            f.write(f" * Saved: {savings:.1f}%\n")
            f.write(f" */\n\n")

            # Write the processed lines
            f.write('\n'.join(processed_lines))
            f.write('\n\n')

            # Close header guard
            f.write(f"#endif /* {guard_name} */\n")

        print(f"Successfully created {header_file}")
        print(f"Size reduction: {original_size:,} -> {minified_size:,} bytes ({savings:.1f}% saved)")

    except Exception as e:
        print(f"Error processing {html_file}: {e}")
    finally:
        # Clean up temporary file
        if 'temp_file_path' in locals() and os.path.exists(temp_file_path):
            os.remove(temp_file_path)

def main():
    # Get all HTML files in current directory
    html_files = [f for f in os.listdir('.') if f.endswith(('.html', '.htm'))]

    if not html_files:
        print("No HTML files found in current directory")
        return

    # Process each HTML file
    for html_file in html_files:
        print(f"\nProcessing {html_file}...")
        convert_file(html_file)

if __name__ == "__main__":
    main()