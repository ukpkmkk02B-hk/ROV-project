import unittest

from modules.comms.surface_comm import extract_line_messages


class SurfaceCommParserTests(unittest.TestCase):
    def test_extract_line_messages_keeps_partial_command_as_leftover(self):
        messages, leftover = extract_line_messages(b'{"type":"command"')

        self.assertEqual(messages, [])
        self.assertEqual(leftover, b'{"type":"command"')

    def test_extract_line_messages_reassembles_split_command(self):
        first_messages, leftover = extract_line_messages(b'{"type":"command"')
        second_messages, leftover = extract_line_messages(leftover + b',"data":{"rov":"tracking start"}}\n')

        self.assertEqual(first_messages, [])
        self.assertEqual(second_messages, ['{"type":"command","data":{"rov":"tracking start"}}'])
        self.assertEqual(leftover, b"")

    def test_extract_line_messages_returns_multiple_commands_from_one_buffer(self):
        messages, leftover = extract_line_messages(
            b'{"type":"command","data":{"rov":"tracking start"}}\n'
            b'{"type":"command","data":{"rov":"stop"}}\n'
        )

        self.assertEqual(
            messages,
            [
                '{"type":"command","data":{"rov":"tracking start"}}',
                '{"type":"command","data":{"rov":"stop"}}',
            ],
        )
        self.assertEqual(leftover, b"")

    def test_extract_line_messages_ignores_empty_lines(self):
        messages, leftover = extract_line_messages(b"\n\r\n  \n{\"bad\": true}\n")

        self.assertEqual(messages, ['{"bad": true}'])
        self.assertEqual(leftover, b"")


if __name__ == "__main__":
    unittest.main()
