<?php
/* SVN FILE: $Id: session.test.php,v 1.1 2007-05-25 05:54:25 rflint%ryanflint.com Exp $ */
/**
 * Short description for file.
 *
 * Long description for file
 *
 * PHP versions 4 and 5
 *
 * CakePHP Test Suite <https://trac.cakephp.org/wiki/Developement/TestSuite>
 * Copyright (c) 2006, Larry E. Masters Shorewood, IL. 60431
 * Author(s): Larry E. Masters aka PhpNut <phpnut@gmail.com>
 *
 *  Licensed under The Open Group Test Suite License
 *  Redistributions of files must retain the above copyright notice.
 *
 * @filesource
 * @author       Larry E. Masters aka PhpNut <phpnut@gmail.com>
 * @copyright    Copyright (c) 2006, Larry E. Masters Shorewood, IL. 60431
 * @link         http://www.phpnut.com/projects/
 * @package      test_suite
 * @subpackage   test_suite.cases.app
 * @since        CakePHP Test Suite v 1.0.0.0
 * @version      $Revision: 1.1 $
 * @modifiedby   $LastChangedBy: phpnut $
 * @lastmodified $Date: 2007-05-25 05:54:25 $
 * @license      http://www.opensource.org/licenses/opengroup.php The Open Group Test Suite License
 */
	require_once LIBS.'session.php';
/**
 * Short description for class.
 *
 * @package    test_suite
 * @subpackage test_suite.cases.libs
 * @since      CakePHP Test Suite v 1.0.0.0
 */
class SessionTest extends UnitTestCase {

	function setUp() {
		restore_error_handler();
		
		@$this->Session =& new CakeSession();
		
		set_error_handler('simpleTestErrorHandler');
	}
	
	function testCheck() {
		$this->Session->write('SessionTestCase', 'value');
		$this->assertTrue($this->Session->check('SessionTestCase'));
		
		$this->assertFalse($this->Session->check('NotExistingSessionTestCase'), false);
	}
	
	function testCheckingSavedEmpty() {
		$this->Session->write('SessionTestCase', 0);
		$this->assertTrue($this->Session->check('SessionTestCase'));
		
		$this->Session->write('SessionTestCase', '0');
		$this->assertTrue($this->Session->check('SessionTestCase'));
		
		$this->Session->write('SessionTestCase', false);
		$this->assertTrue($this->Session->check('SessionTestCase'));
		
		$this->Session->write('SessionTestCase', null);
		$this->assertFalse($this->Session->check('SessionTestCase'));
	}
	
	function testReadingSavedEmpty() {
		$this->Session->write('SessionTestCase', 0);
		$this->assertEqual($this->Session->read('SessionTestCase'), 0);
		
		$this->Session->write('SessionTestCase', '0');
		$this->assertEqual($this->Session->read('SessionTestCase'), '0');
		$this->assertFalse($this->Session->read('SessionTestCase') === 0);
		
		$this->Session->write('SessionTestCase', false);
		$this->assertFalse($this->Session->read('SessionTestCase'));
		
		$this->Session->write('SessionTestCase', null);
		$this->assertEqual($this->Session->read('SessionTestCase'), null);
	}
	
	function tearDown() {
		$this->Session->del('SessionTestCase');
		unset($this->Session);
	}
}

?>